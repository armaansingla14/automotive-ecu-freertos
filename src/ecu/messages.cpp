#include "ecu/messages.hpp"

namespace ecu {

uint16_t crc16_ccitt(const uint8_t* d, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; ++i) {
    crc ^= static_cast<uint16_t>(d[i] << 8);
    for (int b = 0; b < 8; ++b) crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

namespace {
void put_crc(canfd::Frame& f) {
  const uint16_t c = crc16_ccitt(f.data.data(), f.len - 2u);
  f.data[f.len - 2u] = static_cast<uint8_t>(c >> 8);
  f.data[f.len - 1u] = static_cast<uint8_t>(c);
}
bool crc_valid(const canfd::Frame& f) {
  const uint16_t c = crc16_ccitt(f.data.data(), f.len - 2u);
  return f.data[f.len - 2u] == static_cast<uint8_t>(c >> 8) && f.data[f.len - 1u] == static_cast<uint8_t>(c);
}
}  // namespace

canfd::Frame pack(uint32_t id, const SensorMsg& m) {
  canfd::Frame f;
  f.id = id;
  f.len = 64;
  std::memcpy(&f.data[0], &m.seq, 4);
  std::memcpy(&f.data[4], &m.t_us, 8);
  std::memcpy(&f.data[12], m.value, 12);
  f.data[24] = m.flags;
  for (int i = 25; i < 62; ++i) f.data[static_cast<size_t>(i)] = static_cast<uint8_t>(0xA5 ^ i);
  put_crc(f);
  return f;
}

canfd::Frame pack(uint32_t id, const CommandMsg& m) {
  canfd::Frame f;
  f.id = id;
  f.len = 16;
  std::memcpy(&f.data[0], &m.seq, 4);
  std::memcpy(&f.data[4], &m.t_us, 8);
  const int16_t sp = static_cast<int16_t>(m.setpoint * 100.f);
  std::memcpy(&f.data[12], &sp, 2);
  put_crc(f);
  return f;
}

bool unpack(const canfd::Frame& f, SensorMsg& m) {
  if (f.len != 64 || !crc_valid(f)) return false;
  std::memcpy(&m.seq, &f.data[0], 4);
  std::memcpy(&m.t_us, &f.data[4], 8);
  std::memcpy(m.value, &f.data[12], 12);
  m.flags = f.data[24];
  return true;
}

bool unpack(const canfd::Frame& f, CommandMsg& m) {
  if (f.len != 16 || !crc_valid(f)) return false;
  std::memcpy(&m.seq, &f.data[0], 4);
  std::memcpy(&m.t_us, &f.data[4], 8);
  int16_t sp;
  std::memcpy(&sp, &f.data[12], 2);
  m.setpoint = static_cast<float>(sp) / 100.f;
  return true;
}

canfd::Frame pack_diag(uint32_t id, const uint8_t* uds, size_t n) {
  canfd::Frame f;
  f.id = id;
  if (n > 62) n = 62;
  f.data[0] = static_cast<uint8_t>(n);
  std::memcpy(&f.data[1], uds, n);
  f.len = canfd::round_up_len(static_cast<uint8_t>(n + 1));
  for (size_t i = n + 1; i < f.len; ++i) f.data[i] = 0xCC;  // padding
  return f;
}

size_t unpack_diag(const canfd::Frame& f, uint8_t* out, size_t max) {
  const size_t n = f.data[0];
  if (n == 0 || n + 1 > f.len || n > max) return 0;
  std::memcpy(out, &f.data[1], n);
  return n;
}

}  // namespace ecu
