#pragma once
#include <cstdint>
#include <vector>

#include "canfd/bus.hpp"
#include "ecu/dtc.hpp"
#include "ecu/metrics.hpp"
#include "ecu/sensors.hpp"
#include "ecu/uds.hpp"
#include "osal/osal.h"

namespace ecu {

/// Watchdog strategy of the firmware.
enum class WdgMode : uint8_t {
  Off,            ///< no hardware watchdog (baseline)
  Unconditional,  ///< IWDG kicked periodically by the highest-priority task, no supervision
  Supervised,     ///< kick only if every task's heartbeat is fresh; otherwise let the IWDG expire
  SupervisedFast, ///< as Supervised, but trigger a software reset immediately on detection
};
const char* to_string(WdgMode m);

struct EcuConfig {
  int node = 0;
  uint32_t sensor_period_ms = 20;
  uint32_t sensor_id = 0x100;
  uint32_t cmd_period_ms = 50;
  uint32_t cmd_id = 0x080;
  std::vector<uint32_t> rx_sensor_ids;
  std::vector<int> rx_sensor_streams;
  uint32_t rx_cmd_id = 0;
  int rx_cmd_stream = -1;
  uint32_t diag_req_id = 0x700;
  uint32_t diag_resp_id = 0x740;
  int sensor_stream = -1;
  int cmd_stream = -1;
  WdgMode wdg = WdgMode::Supervised;
  /// When all TX buffers are busy, cancel a pending lower-priority frame
  /// (e.g. a diagnostic response) instead of waiting behind it.
  bool tx_cancel = true;
  uint32_t wdg_timeout_ms = 100;
  uint32_t wdg_period_ms = 20;
  uint64_t seed = 1;
};

/// State that survives an MCU reset (emulated NVM / no-init RAM).
struct Nvm {
  DtcManager dtc;
  uint32_t sensor_seq = 0;
  uint32_t cmd_seq = 0;
  uint32_t boots = 0;
  int last_culprit = -1;  ///< task index blamed by the supervisor before a reset
};

/// ECU firmware image. boot() is the reset vector: it creates the RTOS
/// queues and tasks through the OSAL and enables the CAN controller.
///
/// Tasks (priority, period):
///   Watchdog  6  20 ms     heartbeat supervision, IWDG refresh
///   CanRx     5  event     RX FIFO -> dispatch (sensor data, commands, diagnostics)
///   CanTx     5  event     TX queue -> controller TX buffers, stale frames dropped
///   Sensor    4  10..100   sample, plausibility + DTCs, broadcast 64-byte frame
///   Control   3  50 ms     compute and send command to the next ECU
///   Actuator  3  10 ms     apply commands with rate limit, timeout -> safe state
///   Diag      2  event     UDS server (0x10 0x11 0x14 0x19 0x22 0x3E)
class EcuFirmware {
 public:
  enum TaskIdx { kWdg, kRx, kTx, kSensor, kControl, kActuator, kDiag, kNumTasks };
  struct Stats {
    uint64_t tx_queue_full = 0;
    uint64_t stale_dropped = 0;
    uint64_t rx_e2e_fail = 0;
    uint64_t wdg_kicks = 0;
    uint64_t supervision_failures = 0;
    uint64_t rx_overflow = 0;
  };

  EcuFirmware(const EcuConfig& cfg, canfd::Controller& can, Metrics* metrics);
  void boot();
  void on_reset();  ///< hardware side of a reset (CAN controller held in reset)

  // fault injection (transient: cleared by a reset)
  void inject_hang(int task) { hang_[task] = true; }
  void inject_deadlock(int task) { deadlock_[task] = true; }
  SensorModel& sensors() { return sensors_; }

  Nvm& nvm() { return nvm_; }
  const EcuConfig& config() const { return cfg_; }
  const Stats& stats() const { return stats_; }
  const Actuator& actuator() const { return act_; }
  uint64_t boot_time_us() const { return boot_us_; }

 private:
  struct TxItem {
    canfd::Frame frame;
    uint32_t deadline_ms;
  };
  static void wdg_task(void* p);
  static void rx_task(void* p);
  static void tx_task(void* p);
  static void sensor_task(void* p);
  static void control_task(void* p);
  static void actuator_task(void* p);
  static void diag_task(void* p);

  void heartbeat(int t) { hb_[t] = osal_tick_ms(); }
  void check_faults(int t);
  void dispatch(const canfd::Frame& f);
  bool read_did(uint16_t did, std::vector<uint8_t>& out);
  DtcManager::Snapshot snapshot();
  bool enqueue_tx(const canfd::Frame& f, uint32_t deadline_ms);

  EcuConfig cfg_;
  canfd::Controller& can_;
  Metrics* metrics_;
  Nvm nvm_;
  Stats stats_;
  SensorModel sensors_;
  Plausibility plaus_[3];
  Actuator act_;
  UdsServer uds_;

  osal_queue_t tx_q_ = nullptr, rx_q_ = nullptr, act_q_ = nullptr, diag_q_ = nullptr, dead_q_ = nullptr;
  uint32_t hb_[kNumTasks] = {};
  uint32_t hb_limit_[kNumTasks] = {};
  bool hang_[kNumTasks] = {};
  bool deadlock_[kNumTasks] = {};
  float local_[3] = {0, 0, 0};
  float remote_[3] = {0, 0, 0};
  uint64_t boot_us_ = 0;
  uint32_t boot_ms_ = 0;
};

}  // namespace ecu
