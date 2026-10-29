#pragma once
#include <cstdint>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "canfd/bus.hpp"
#include "ecu/ecu_node.hpp"
#include "ecu/metrics.hpp"

namespace ecu {

struct NetConfig {
  int nodes = 20;
  canfd::BitTiming timing{};
  double ber = 0.0;
  double background_load = 0.0;  ///< extra bus load from non-ECU traffic (fraction of capacity)
  bool deadline_monotonic_ids = true;
  WdgMode wdg = WdgMode::Supervised;
  bool tester = true;             ///< diagnostic tester polling every 100 ms
  bool tx_cancel = true;          ///< ECU TX driver cancels lower-priority pending frames
  uint64_t seed = 1;
};

/// A diagnostic tester on the bus (outside the 20 ECUs) that polls each ECU
/// in turn with UDS requests and checks the responses.
struct TesterStats {
  uint64_t requests = 0;
  uint64_t positive = 0;
  uint64_t negative = 0;
  uint64_t no_response = 0;
  std::vector<double> response_ms;
};

/// 20-node CAN FD network: ECU firmware on the simulation kernel, the bus,
/// an optional background traffic source and a diagnostic tester.
class Network {
 public:
  explicit Network(const NetConfig& cfg);
  ~Network();

  void run_until_ms(uint64_t t_ms);
  uint64_t now_ms() const;

  /// Fault injection helpers (scheduled in simulated time).
  void schedule_hang(uint64_t t_ms, int node, int task);
  void schedule_deadlock(uint64_t t_ms, int node, int task);
  void schedule_tx_fault(uint64_t t0_ms, uint64_t t1_ms, int node);
  void add_noise_window(uint64_t t0_ms, uint64_t t1_ms, double ber) { bus_->add_noise_window(t0_ms * 1000, t1_ms * 1000, ber); }

  Metrics& metrics() { return metrics_; }
  canfd::Bus& bus() { return *bus_; }
  EcuFirmware& ecu(int i) { return *ecus_[static_cast<size_t>(i)]; }
  int size() const { return static_cast<int>(ecus_.size()); }
  const TesterStats& tester() const { return tester_; }
  uint64_t background_dropped() const { return bg_dropped_; }
  uint64_t background_sent() const { return bg_sent_; }
  const NetConfig& config() const { return cfg_; }

  static uint32_t sensor_period_for(int node);

 private:
  void start_background();
  void bg_send(int stream, uint64_t period_us);
  void tester_poll();

  NetConfig cfg_;
  Metrics metrics_;
  std::unique_ptr<canfd::Bus> bus_;
  std::vector<std::unique_ptr<EcuFirmware>> ecus_;
  canfd::Controller* bg_ctrl_ = nullptr;
  canfd::Controller* tester_ctrl_ = nullptr;
  std::mt19937_64 rng_;
  uint64_t bg_sent_ = 0, bg_dropped_ = 0;
  TesterStats tester_;
  int tester_target_ = 0;
  int tester_step_ = 0;
  bool tester_waiting_ = false;
  uint64_t tester_sent_us_ = 0;
};

}  // namespace ecu
