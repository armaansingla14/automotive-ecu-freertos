#include "ecu/sensors.hpp"

#include <algorithm>
#include <cmath>

namespace ecu {

SensorModel::SensorModel(uint64_t seed, float phase) : rng_(static_cast<uint32_t>(seed)), phase_(phase) {}

void SensorModel::sample(uint32_t t_ms, float out[3]) {
  std::normal_distribution<float> noise(0.f, 0.02f);
  const float t = static_cast<float>(t_ms) * 1e-3f;
  // nominal signals in volts (0.5..4.5 V sensor range)
  const float nominal[3] = {
      2.5f + 1.2f * std::sin(0.2f * t + phase_),
      1.8f + 0.6f * std::sin(0.05f * t + 2.f * phase_),
      3.0f + 0.9f * std::sin(0.7f * t + 0.5f * phase_),
  };
  for (int i = 0; i < 3; ++i) {
    float v = nominal[i] + noise(rng_);
    switch (fault_[i]) {
      case Fault::None: stuck_latched_[i] = false; break;
      case Fault::OpenCircuit: v = 4.98f; break;  // pull-up to rail
      case Fault::Stuck:
        if (!stuck_latched_[i]) {
          stuck_latched_[i] = true;
          stuck_value_[i] = v;
        }
        v = stuck_value_[i];
        break;
      case Fault::Spike:
        if (std::uniform_int_distribution<int>(0, 9)(rng_) == 0) v += 1.5f;
        break;
    }
    out[i] = v;
  }
}

Plausibility::Result Plausibility::check(float v) {
  Result r{false, false, false};
  r.range_fail = v < lim_.min || v > lim_.max;
  if (have_prev_) {
    r.rate_fail = std::fabs(v - prev_) > lim_.max_rate;
    same_ = (v == prev_) ? same_ + 1 : 0;
    r.stuck_fail = same_ >= lim_.stuck_samples;
  }
  prev_ = v;
  have_prev_ = true;
  return r;
}

void Actuator::command(float sp, uint32_t now_ms) {
  target_ = std::min(std::max(sp, cfg_.min), cfg_.max);
  last_cmd_ms_ = now_ms;
  have_cmd_ = true;
  timed_out_ = false;
}

void Actuator::update(uint32_t now_ms) {
  if (have_cmd_ && !timed_out_ && now_ms - last_cmd_ms_ > cfg_.timeout_ms) {
    timed_out_ = true;
    ++timeouts_;
  }
  const float goal = (timed_out_ || !have_cmd_) ? cfg_.safe_value : target_;
  const float dt = static_cast<float>(now_ms - last_update_ms_);
  last_update_ms_ = now_ms;
  const float step = cfg_.rate_per_ms * dt;
  if (goal > out_) {
    out_ = std::min(goal, out_ + step);
  } else {
    out_ = std::max(goal, out_ - step);
  }
}

}  // namespace ecu
