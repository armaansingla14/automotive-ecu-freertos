#pragma once
#include <cstdint>
#include <functional>
#include <vector>

#include "ecu/dtc.hpp"

namespace ecu {

namespace uds {
constexpr uint8_t kDiagSessionControl = 0x10;
constexpr uint8_t kEcuReset = 0x11;
constexpr uint8_t kClearDtc = 0x14;
constexpr uint8_t kReadDtc = 0x19;
constexpr uint8_t kReadDid = 0x22;
constexpr uint8_t kTesterPresent = 0x3E;
constexpr uint8_t kNegative = 0x7F;

constexpr uint8_t kNrcServiceNotSupported = 0x11;
constexpr uint8_t kNrcSubFunctionNotSupported = 0x12;
constexpr uint8_t kNrcIncorrectLength = 0x13;
constexpr uint8_t kNrcRequestOutOfRange = 0x31;

constexpr uint8_t kReportDtcByStatusMask = 0x02;
constexpr uint8_t kReportSnapshotByDtc = 0x04;
constexpr uint8_t kReportNumberOfDtcByMask = 0x01;

constexpr uint16_t kDidNodeId = 0xF18C;      // ECU serial / node id
constexpr uint16_t kDidResetCount = 0x0101;
constexpr uint16_t kDidUptime = 0x0102;
constexpr uint16_t kDidSensorValues = 0x0103;
constexpr uint16_t kDidResetCause = 0x0104;
}  // namespace uds

/// UDS (ISO 14229-1) server subset: 0x10, 0x11, 0x14, 0x19 (01/02/04),
/// 0x22, 0x3E. Requests and responses are raw service bytes (the transport
/// is a single CAN FD frame, see messages.hpp).
class UdsServer {
 public:
  using DidReader = std::function<bool(uint16_t did, std::vector<uint8_t>& out)>;
  UdsServer(DtcManager& dtc, DidReader dids) : dtc_(dtc), dids_(std::move(dids)) {}

  /// Returns the response; empty for suppressed positive responses.
  std::vector<uint8_t> handle(const std::vector<uint8_t>& req);
  bool reset_requested() const { return reset_requested_; }
  void on_boot() {
    reset_requested_ = false;
    session_ = 1;
  }
  uint8_t session() const { return session_; }

 private:
  static std::vector<uint8_t> nrc(uint8_t sid, uint8_t code) { return {uds::kNegative, sid, code}; }
  DtcManager& dtc_;
  DidReader dids_;
  bool reset_requested_ = false;
  uint8_t session_ = 1;
};

}  // namespace ecu
