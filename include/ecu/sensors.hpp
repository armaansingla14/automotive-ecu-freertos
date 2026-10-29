#pragma once
#include <cstdint>
#include <random>

namespace ecu {

/// Synthetic sensor front end for one ECU: three analogue channels
/// (e.g. temperature, pressure, position) with noise, plus supply voltage.
/// Faults can be injected per channel.
class SensorModel {
 public:
  enum class Fault : uint8_t { None, OpenCircuit, Stuck, Spike };
  explicit SensorModel(uint64_t seed, float phase = 0.f);
  /// Sample all channels at time t (ms).
  void sample(uint32_t t_ms, float out[3]);
  float supply_voltage() const { return 13.8f; }
  void set_fault(int ch, Fault f) { fault_[ch] = f; }
  Fault fault(int ch) const { return fault_[ch]; }

 private:
  std::mt19937 rng_;
  float phase_;
  Fault fault_[3] = {Fault::None, Fault::None, Fault::None};
  float stuck_value_[3] = {0, 0, 0};
  bool stuck_latched_[3] = {false, false, false};
};

/// Signal plausibility monitors evaluated every sample.
class Plausibility {
 public:
  struct Limits {
    float min, max;     ///< valid electrical range
    float max_rate;     ///< max change per sample
    int stuck_samples;  ///< identical samples considered stuck
  };
  explicit Plausibility(const Limits& l) : lim_(l) {}
  struct Result {
    bool range_fail, rate_fail, stuck_fail;
  };
  Result check(float v);

 private:
  Limits lim_;
  bool have_prev_ = false;
  float prev_ = 0;
  int same_ = 0;
};

/// Rate-limited actuator (e.g. a valve or motor setpoint) with command
/// timeout supervision: without a fresh command it ramps to a safe value.
class Actuator {
 public:
  struct Config {
    float min = 0.f, max = 100.f;
    float rate_per_ms = 0.5f;
    float safe_value = 0.f;
    uint32_t timeout_ms = 150;
  };
  Actuator() = default;
  explicit Actuator(const Config& c) : cfg_(c) {}
  void command(float setpoint, uint32_t now_ms);
  void update(uint32_t now_ms);
  float output() const { return out_; }
  bool timed_out() const { return timed_out_; }
  uint32_t timeouts() const { return timeouts_; }

 private:
  Config cfg_{};
  float target_ = 0.f;
  float out_ = 0.f;
  uint32_t last_cmd_ms_ = 0;
  uint32_t last_update_ms_ = 0;
  bool have_cmd_ = false;
  bool timed_out_ = false;
  uint32_t timeouts_ = 0;
};

}  // namespace ecu
