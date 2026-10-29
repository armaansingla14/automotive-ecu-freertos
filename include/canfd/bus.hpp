#pragma once
#include <cstdint>
#include <functional>
#include <memory>
#include <random>
#include <vector>

#include "canfd/frame.hpp"
#include "sim/kernel.hpp"

namespace canfd {

enum class ErrState : uint8_t { Active, Passive, BusOff };

/// CAN FD controller model (FDCAN-like): a few TX buffers sent in ID order,
/// acceptance filters, ISO 11898-1 fault confinement (TEC/REC, error
/// passive, bus-off with automatic recovery after 128 x 11 recessive bits).
class Controller {
 public:
  struct Pending {
    Frame frame;
    uint64_t submit_us;
  };
  using RxIsr = std::function<void(const Frame&, uint64_t t_us)>;
  using TxDone = std::function<void(const Frame&, uint64_t t_us)>;

  int node = -1;
  RxIsr rx_isr;
  TxDone tx_done;
  bool enabled = true;      ///< false while the MCU is in reset
  bool tx_fault = false;    ///< faulty transceiver: own transmissions corrupted
  size_t tx_buffers = 3;

  /// Acceptance filter (id & mask) == (filter & mask); empty = accept all.
  void add_filter(uint32_t id, uint32_t mask) { filters_.push_back({id, mask}); }
  bool accepts(uint32_t id) const;

  bool submit(const Frame& f, uint64_t now_us);  ///< false if all TX buffers busy or bus-off
  size_t free_buffers() const { return tx_buffers - pending_.size(); }
  void flush();  ///< drop all pending (reset / bus-off)
  /// TX buffer cancellation request: drop the lowest-priority pending frame
  /// if its ID is numerically higher than `id`. Returns true on success.
  bool cancel_lower_priority(uint32_t id);

  ErrState state() const { return state_; }
  uint16_t tec() const { return tec_; }
  uint16_t rec() const { return rec_; }

  // statistics
  uint64_t tx_ok = 0, tx_errors = 0, rx_ok = 0, bus_off_events = 0, frames_flushed = 0, tx_cancelled = 0;

 private:
  friend class Bus;
  struct Filter {
    uint32_t id, mask;
  };
  std::vector<Filter> filters_;
  std::vector<Pending> pending_;
  ErrState state_ = ErrState::Active;
  uint16_t tec_ = 0;
  uint16_t rec_ = 0;
  uint64_t suspend_until_us_ = 0;  ///< error-passive suspend transmission
  class Bus* bus_ = nullptr;
  void update_state();
};

struct BusConfig {
  BitTiming timing{};
  double ber = 0.0;      ///< random bit error rate on the wire
  uint64_t seed = 1;
};

/// Shared CAN FD bus with bitwise arbitration (lowest ID wins), bit-error
/// injection, error frames and automatic retransmission.
class Bus {
 public:
  Bus(sim::Kernel& k, const BusConfig& cfg);
  Controller& attach(int node_id);
  Controller& controller(size_t i) { return *ctrls_[i]; }
  size_t controllers() const { return ctrls_.size(); }

  void set_ber(double ber) { cfg_.ber = ber; }
  /// Elevated bit error rate in [t0, t1) (EMI burst).
  void add_noise_window(uint64_t t0_us, uint64_t t1_us, double ber);

  void kick();  ///< a controller has something new to send

  // statistics
  uint64_t busy_ns = 0;
  uint64_t frames_ok = 0;
  uint64_t error_frames = 0;
  uint64_t undetected_errors = 0;
  uint64_t arbitration_losses = 0;
  double utilization(uint64_t window_us) const {
    return window_us ? static_cast<double>(busy_ns) / (1000.0 * static_cast<double>(window_us)) : 0.0;
  }
  const BusConfig& config() const { return cfg_; }

 private:
  struct Noise {
    uint64_t t0, t1;
    double ber;
  };
  void start_next();
  double ber_at(uint64_t t_us) const;

  sim::Kernel& k_;
  BusConfig cfg_;
  std::vector<std::unique_ptr<Controller>> ctrls_;
  std::vector<Noise> noise_;
  std::mt19937_64 rng_;
  bool busy_ = false;
  bool start_scheduled_ = false;
  uint64_t busy_until_ns_ = 0;
};

}  // namespace canfd
