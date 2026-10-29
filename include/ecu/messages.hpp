#pragma once
#include <cstdint>
#include <cstring>

#include "canfd/frame.hpp"

namespace ecu {

/// CRC-16/CCITT-FALSE, used as an application-level end-to-end check on
/// top of the CAN FD link CRC.
uint16_t crc16_ccitt(const uint8_t* d, size_t n);

/// Sensor broadcast: 64-byte CAN FD payload.
///  [0..3] sequence  [4..11] producer timestamp (us, measurement only)
///  [12..23] 3 x float sensor  [24] status flags  [25..61] reserved/fill
///  [62..63] CRC-16
struct SensorMsg {
  uint32_t seq;
  uint64_t t_us;
  float value[3];
  uint8_t flags;
};

/// Command to the next ECU: 16-byte payload.
///  [0..3] seq  [4..11] timestamp  [12..13] setpoint x100  [14..15] CRC-16
struct CommandMsg {
  uint32_t seq;
  uint64_t t_us;
  float setpoint;
};

canfd::Frame pack(uint32_t id, const SensorMsg& m);
canfd::Frame pack(uint32_t id, const CommandMsg& m);
bool unpack(const canfd::Frame& f, SensorMsg& m);   ///< false on length/CRC error
bool unpack(const canfd::Frame& f, CommandMsg& m);

/// Diagnostic frames: [0] = UDS length (1..62), [1..] UDS bytes.
canfd::Frame pack_diag(uint32_t id, const uint8_t* uds, size_t n);
size_t unpack_diag(const canfd::Frame& f, uint8_t* out, size_t max);

}  // namespace ecu
