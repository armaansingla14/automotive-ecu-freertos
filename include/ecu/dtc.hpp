#pragma once
#include <array>
#include <cstdint>
#include <vector>

namespace ecu {

/// ISO 14229 DTC status bits (subset).
namespace dtc_status {
constexpr uint8_t kTestFailed = 0x01;
constexpr uint8_t kTestFailedThisCycle = 0x02;
constexpr uint8_t kPending = 0x04;
constexpr uint8_t kConfirmed = 0x08;
constexpr uint8_t kAvailabilityMask = 0x0F;
}  // namespace dtc_status

/// 24-bit DTC numbers used by the ECU (SAE J2012 style, Uxxxx/Pxxxx).
enum class DtcId : uint32_t {
  SensorRange = 0x012300,    // P0123-like: sensor out of range
  SensorStuck = 0x012400,    // sensor signal stuck
  SensorRate = 0x012500,     // implausible rate of change
  CommandTimeout = 0xC10000, // U0100-like: lost communication with commanding node
  CanBusOff = 0xC07300,      // U0073-like: CAN bus off
  WatchdogReset = 0xD00000,  // ECU internal: watchdog reset occurred
};

/// Snapshot stored with the first confirmation of a DTC.
struct FreezeFrame {
  bool valid = false;
  uint32_t timestamp_ms = 0;  ///< ECU uptime at capture
  uint32_t odometer = 0;      ///< operation-cycle counter as a stand-in
  float sensor[3] = {0, 0, 0};
  float supply_v = 0;
  uint8_t occurrence = 0;
};

/// Fault memory with debouncing (counter-based, ISO 14229-like status
/// handling) and one freeze frame per DTC. Lives in "NVM": survives resets.
class DtcManager {
 public:
  struct Entry {
    DtcId id;
    uint8_t status = 0;
    int8_t counter = 0;  ///< debounce counter, fails at +threshold, passes at -threshold
    uint8_t occurrences = 0;
    FreezeFrame ff;
  };
  struct Snapshot {
    uint32_t now_ms;
    float sensor[3];
    float supply_v;
  };

  explicit DtcManager(int8_t threshold = 3);

  /// Report one test result. failed=true counts up, false counts down.
  void report(DtcId id, bool failed, const Snapshot& snap);
  /// Immediate failure without debouncing (events such as a reset).
  void report_event(DtcId id, const Snapshot& snap);
  void clear_all();
  void new_operation_cycle();

  const Entry* find(DtcId id) const;
  uint8_t status(DtcId id) const;
  std::vector<const Entry*> by_status_mask(uint8_t mask) const;
  const std::vector<Entry>& entries() const { return entries_; }
  uint32_t operation_cycles() const { return cycles_; }

 private:
  Entry& get(DtcId id);
  void fail(Entry& e, const Snapshot& snap);
  int8_t threshold_;
  std::vector<Entry> entries_;
  uint32_t cycles_ = 0;
};

}  // namespace ecu
