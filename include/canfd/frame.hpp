#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace canfd {

/// CAN FD data frame (11-bit base identifier, FDF = 1).
struct Frame {
  uint32_t id = 0;
  uint8_t len = 0;   ///< payload length in bytes: 0..8, 12, 16, 20, 24, 32, 48, 64
  bool brs = true;   ///< bit-rate switch for the data phase
  std::array<uint8_t, 64> data{};
};

uint8_t len_to_dlc(uint8_t len);   ///< smallest DLC that holds len bytes
uint8_t dlc_to_len(uint8_t dlc);
uint8_t round_up_len(uint8_t len); ///< next valid CAN FD payload length

/// CRC-17 / CRC-21 as specified for CAN FD (ISO 11898-1:2015), MSB-first,
/// register initialised with a leading 1.
uint32_t crc17(const std::vector<uint8_t>& bits, size_t begin, size_t end);
uint32_t crc21(const std::vector<uint8_t>& bits, size_t begin, size_t end);

/// Bit-level image of a frame on the wire, used for timing (exact dynamic
/// bit-stuffing count) and for error injection (the receiver re-computes
/// the CRC over the possibly corrupted bits).
struct WireImage {
  std::vector<uint8_t> bits;  ///< unstuffed bits SOF .. CRC (one byte per bit)
  size_t arb_bits = 0;        ///< SOF..BRS, sent at the nominal rate
  size_t crc_begin = 0;       ///< index of the first CRC bit
  int crc_len = 0;            ///< 17 or 21
  int dyn_stuff_arb = 0;      ///< dynamic stuff bits in the arbitration phase
  int dyn_stuff_data = 0;     ///< dynamic stuff bits in the data phase
  int fixed_stuff = 0;        ///< fixed stuff bits in stuff-count + CRC field
};

WireImage encode(const Frame& f);
/// CRC over the (possibly modified) wire bits matches the CRC field?
bool crc_ok(const WireImage& w);
/// Payload as seen by a receiver of the given wire bits.
Frame decode(const WireImage& w, const Frame& original);

struct BitTiming {
  uint32_t nominal_bps = 500000;
  uint32_t data_bps = 2000000;
};

/// Total bus time of the frame including ACK, EOF and intermission (ns).
uint64_t frame_duration_ns(const WireImage& w, bool brs, const BitTiming& bt);
/// Number of bit times on the wire (for BER calculations).
size_t wire_bits(const WireImage& w);

}  // namespace canfd
