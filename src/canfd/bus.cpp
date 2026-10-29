#include "canfd/bus.hpp"

#include <algorithm>
#include <cmath>

namespace canfd {

bool Controller::accepts(uint32_t id) const {
  if (filters_.empty()) return true;
  for (const auto& f : filters_) {
    if ((id & f.mask) == (f.id & f.mask)) return true;
  }
  return false;
}

bool Controller::submit(const Frame& f, uint64_t now_us) {
  if (!enabled || state_ == ErrState::BusOff || pending_.size() >= tx_buffers) return false;
  pending_.push_back({f, now_us});
  if (bus_) bus_->kick();
  return true;
}

void Controller::flush() {
  frames_flushed += pending_.size();
  pending_.clear();
}

bool Controller::cancel_lower_priority(uint32_t id) {
  auto it = std::max_element(pending_.begin(), pending_.end(),
                             [](const Pending& a, const Pending& b) { return a.frame.id < b.frame.id; });
  if (it == pending_.end() || it->frame.id <= id) return false;
  pending_.erase(it);
  ++tx_cancelled;
  return true;
}

void Controller::update_state() {
  if (tec_ > 255) {
    state_ = ErrState::BusOff;
  } else if (tec_ >= 128 || rec_ >= 128) {
    state_ = ErrState::Passive;
  } else {
    state_ = ErrState::Active;
  }
}

Bus::Bus(sim::Kernel& k, const BusConfig& cfg) : k_(k), cfg_(cfg), rng_(cfg.seed) {}

Controller& Bus::attach(int node_id) {
  auto c = std::make_unique<Controller>();
  c->node = node_id;
  c->bus_ = this;
  ctrls_.push_back(std::move(c));
  return *ctrls_.back();
}

void Bus::add_noise_window(uint64_t t0, uint64_t t1, double ber) { noise_.push_back({t0, t1, ber}); }

double Bus::ber_at(uint64_t t) const {
  double b = cfg_.ber;
  for (const auto& n : noise_) {
    if (t >= n.t0 && t < n.t1) b = std::max(b, n.ber);
  }
  return b;
}

void Bus::kick() {
  if (busy_ || start_scheduled_) return;
  start_scheduled_ = true;
  // arbitration happens at the next bus-idle instant; all requests made at
  // the same instant compete
  k_.at(k_.now_us(), [this]() {
    start_scheduled_ = false;
    start_next();
  });
}

void Bus::start_next() {
  if (busy_) return;
  const uint64_t now = k_.now_us();
  Controller* winner = nullptr;
  size_t widx = 0;
  int contenders = 0;
  uint64_t earliest_suspend = sim::kNever;
  for (auto& c : ctrls_) {
    if (!c->enabled || c->state_ == ErrState::BusOff || c->pending_.empty()) continue;
    if (c->suspend_until_us_ > now) {
      earliest_suspend = std::min(earliest_suspend, c->suspend_until_us_);
      continue;
    }
    // each controller offers its highest-priority (lowest ID) buffer
    size_t best = 0;
    for (size_t i = 1; i < c->pending_.size(); ++i) {
      if (c->pending_[i].frame.id < c->pending_[best].frame.id) best = i;
    }
    ++contenders;
    if (!winner || c->pending_[best].frame.id < winner->pending_[widx].frame.id) {
      winner = c.get();
      widx = best;
    }
  }
  if (!winner) {
    if (earliest_suspend != sim::kNever) k_.at(earliest_suspend, [this]() { start_next(); });
    return;
  }
  arbitration_losses += static_cast<uint64_t>(contenders - 1);

  const Frame f = winner->pending_[widx].frame;
  WireImage w = encode(f);
  const uint64_t dur_ns = frame_duration_ns(w, f.brs, cfg_.timing);
  const size_t nbits = wire_bits(w);
  const double ber = winner->tx_fault ? 0.05 : ber_at(now);

  // --- error injection ----------------------------------------------------
  bool error = false;
  uint64_t err_time_ns = dur_ns;
  if (ber > 0.0) {
    std::uniform_real_distribution<double> u(0.0, 1.0);
    const double p_frame = 1.0 - std::pow(1.0 - ber, static_cast<double>(nbits));
    if (u(rng_) < p_frame) {
      // position of the first flipped bit, further flips with the same BER
      std::uniform_int_distribution<size_t> pos(0, nbits - 1);
      const size_t first = pos(rng_);
      std::vector<size_t> flips{first};
      std::binomial_distribution<int> more(static_cast<int>(nbits - first - 1), std::min(ber, 1.0));
      const int extra = nbits - first > 1 ? more(rng_) : 0;
      for (int i = 0; i < extra; ++i) {
        flips.push_back(std::uniform_int_distribution<size_t>(first + 1, nbits - 1)(rng_));
      }
      const size_t img_bits = w.bits.size();
      if (first < w.arb_bits + 5) {
        // header/control: the transmitter sees a bit error or receivers a
        // form/stuff error right away
        error = true;
        err_time_ns = dur_ns * first / nbits;
      } else if (first < img_bits) {
        for (size_t p : flips) {
          if (p < img_bits) w.bits[p] ^= 1U;
        }
        if (!crc_ok(w)) {
          error = true;
          err_time_ns = dur_ns * (img_bits + 2) / nbits;  // CRC error flagged after ACK delimiter
        } else {
          ++undetected_errors;  // delivered with corrupted payload
        }
      } else {
        // delimiter / EOF: form error
        error = true;
        err_time_ns = dur_ns * first / nbits;
      }
    }
  }

  busy_ = true;
  const uint64_t tn = 1000000000ULL / cfg_.timing.nominal_bps;
  if (error) {
    // error flag (6) + delimiter (8) + intermission (3)
    const uint64_t total_ns = err_time_ns + 17 * tn;
    busy_ns += total_ns;
    ++error_frames;
    const uint64_t t_end = now + (total_ns + 999) / 1000;
    k_.at(t_end, [this, winner, t_end, tn]() {
      busy_ = false;
      winner->tec_ = static_cast<uint16_t>(winner->tec_ + 8);
      ++winner->tx_errors;
      for (auto& c : ctrls_) {
        if (c.get() != winner && c->enabled && c->state_ != ErrState::BusOff && c->rec_ < 255) ++c->rec_;
        c->update_state();
      }
      winner->update_state();
      if (winner->state_ == ErrState::BusOff) {
        ++winner->bus_off_events;
        winner->flush();
        // automatic recovery after 128 occurrences of 11 recessive bits
        const uint64_t rec_us = (128ULL * 11ULL * tn) / 1000ULL;
        Controller* c = winner;
        k_.at(t_end + rec_us, [this, c]() {
          c->tec_ = c->rec_ = 0;
          c->state_ = ErrState::Active;
          kick();
        });
      } else if (winner->state_ == ErrState::Passive) {
        winner->suspend_until_us_ = t_end + (8 * tn + 999) / 1000;
      }
      start_next();
    });
    return;
  }

  busy_ns += dur_ns;
  const uint64_t t_end = now + (dur_ns + 999) / 1000;
  const Frame rx_frame = decode(w, f);
  winner->pending_.erase(winner->pending_.begin() + static_cast<long>(widx));
  k_.at(t_end, [this, winner, f, rx_frame, t_end, tn]() {
    busy_ = false;
    ++frames_ok;
    if (winner->tec_ > 0) --winner->tec_;
    ++winner->tx_ok;
    const bool was_passive = winner->state_ == ErrState::Passive;
    winner->update_state();
    if (was_passive) winner->suspend_until_us_ = t_end + (8 * tn + 999) / 1000;
    for (auto& c : ctrls_) {
      if (c.get() == winner || !c->enabled || c->state_ == ErrState::BusOff) continue;
      if (c->rec_ > 0) --c->rec_;
      c->update_state();
      if (!c->accepts(rx_frame.id)) continue;
      ++c->rx_ok;
      if (c->rx_isr) c->rx_isr(rx_frame, t_end);
    }
    if (winner->tx_done) winner->tx_done(f, t_end);
    start_next();
  });
}

}  // namespace canfd
