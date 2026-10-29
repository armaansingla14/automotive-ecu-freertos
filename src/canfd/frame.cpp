#include "canfd/frame.hpp"

namespace canfd {

namespace {
const uint8_t kDlcLen[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

void push(std::vector<uint8_t>& b, uint32_t v, int n) {
  for (int i = n - 1; i >= 0; --i) b.push_back(static_cast<uint8_t>((v >> i) & 1U));
}

uint32_t crc_generic(const std::vector<uint8_t>& bits, size_t begin, size_t end, int n, uint32_t poly) {
  const uint32_t mask = (1U << n) - 1U;
  uint32_t crc = 1U << (n - 1);
  for (size_t i = begin; i < end; ++i) {
    const uint32_t nxt = bits[i] ^ ((crc >> (n - 1)) & 1U);
    crc = (crc << 1) & mask;
    if (nxt) crc ^= poly;
  }
  return crc & mask;
}

int count_dynamic_stuff(const std::vector<uint8_t>& bits, size_t begin, size_t end, uint8_t& last, int& run) {
  int stuff = 0;
  for (size_t i = begin; i < end; ++i) {
    if (bits[i] == last) {
      ++run;
    } else {
      last = bits[i];
      run = 1;
    }
    if (run == 5) {
      ++stuff;
      last = static_cast<uint8_t>(!last);  // stuff bit of opposite polarity
      run = 1;
    }
  }
  return stuff;
}

// 3-bit Gray code of (stuff count mod 8) followed by an even parity bit
uint32_t stuff_count_field(int count) {
  static const uint8_t gray[8] = {0, 1, 3, 2, 6, 7, 5, 4};
  const uint32_t g = gray[count & 7];
  const uint32_t parity = ((g >> 2) ^ (g >> 1) ^ g) & 1U;
  return (g << 1) | parity;
}
}  // namespace

uint8_t len_to_dlc(uint8_t len) {
  for (uint8_t d = 0; d < 16; ++d) {
    if (kDlcLen[d] >= len) return d;
  }
  return 15;
}

uint8_t dlc_to_len(uint8_t dlc) { return kDlcLen[dlc & 15]; }
uint8_t round_up_len(uint8_t len) { return dlc_to_len(len_to_dlc(len)); }

uint32_t crc17(const std::vector<uint8_t>& bits, size_t b, size_t e) { return crc_generic(bits, b, e, 17, 0x1685BU); }
uint32_t crc21(const std::vector<uint8_t>& bits, size_t b, size_t e) { return crc_generic(bits, b, e, 21, 0x102899U); }

WireImage encode(const Frame& f) {
  WireImage w;
  auto& b = w.bits;
  const uint8_t len = round_up_len(f.len);
  b.reserve(64 + len * 8u + 32);
  push(b, 0, 1);              // SOF
  push(b, f.id & 0x7FF, 11);  // base ID
  push(b, 0, 1);              // RRS
  push(b, 0, 1);              // IDE
  push(b, 1, 1);              // FDF
  push(b, 0, 1);              // res
  push(b, f.brs ? 1 : 0, 1);  // BRS
  w.arb_bits = b.size();
  push(b, 0, 1);  // ESI (error active)
  push(b, len_to_dlc(len), 4);
  for (uint8_t i = 0; i < len; ++i) push(b, f.data[i], 8);
  const size_t data_end = b.size();

  uint8_t last = 2;
  int run = 0;
  w.dyn_stuff_arb = count_dynamic_stuff(b, 0, w.arb_bits, last, run);
  w.dyn_stuff_data = count_dynamic_stuff(b, w.arb_bits, data_end, last, run);

  push(b, stuff_count_field(w.dyn_stuff_arb + w.dyn_stuff_data), 4);
  w.crc_len = len <= 16 ? 17 : 21;
  w.crc_begin = b.size();
  // simplification: CRC over the unstuffed SOF..stuff-count sequence
  const uint32_t crc = w.crc_len == 17 ? crc17(b, 0, w.crc_begin) : crc21(b, 0, w.crc_begin);
  push(b, crc, w.crc_len);
  // fixed stuff bits: one before the stuff count, then one every 4 bits
  w.fixed_stuff = 1 + (4 + w.crc_len) / 4;
  return w;
}

bool crc_ok(const WireImage& w) {
  const uint32_t calc = w.crc_len == 17 ? crc17(w.bits, 0, w.crc_begin) : crc21(w.bits, 0, w.crc_begin);
  uint32_t rx = 0;
  for (int i = 0; i < w.crc_len; ++i) rx = (rx << 1) | w.bits[w.crc_begin + static_cast<size_t>(i)];
  return calc == rx;
}

Frame decode(const WireImage& w, const Frame& original) {
  Frame f = original;
  uint32_t id = 0;
  for (int i = 1; i <= 11; ++i) id = (id << 1) | w.bits[static_cast<size_t>(i)];
  f.id = id;
  const size_t data0 = w.arb_bits + 1 + 4;
  for (uint8_t i = 0; i < round_up_len(original.len); ++i) {
    uint8_t v = 0;
    for (int k = 0; k < 8; ++k) v = static_cast<uint8_t>((v << 1) | w.bits[data0 + i * 8u + static_cast<size_t>(k)]);
    f.data[i] = v;
  }
  return f;
}

size_t wire_bits(const WireImage& w) {
  // stuffed frame + CRC delimiter, ACK slot, ACK delimiter, EOF (7), IFS (3)
  return w.bits.size() + static_cast<size_t>(w.dyn_stuff_arb + w.dyn_stuff_data + w.fixed_stuff) + 1 + 1 + 1 + 7 + 3;
}

uint64_t frame_duration_ns(const WireImage& w, bool brs, const BitTiming& bt) {
  const uint64_t tn = 1000000000ULL / bt.nominal_bps;
  const uint64_t td = brs ? 1000000000ULL / bt.data_bps : tn;
  const uint64_t arb = (w.arb_bits + static_cast<size_t>(w.dyn_stuff_arb)) * tn;
  const uint64_t data_bits =
      (w.bits.size() - w.arb_bits) + static_cast<size_t>(w.dyn_stuff_data + w.fixed_stuff) + 1;  // + CRC delimiter
  const uint64_t tail = (1 + 1 + 7 + 3) * tn;  // ACK slot, ACK delim, EOF, IFS
  return arb + data_bits * td + tail;
}

}  // namespace canfd
