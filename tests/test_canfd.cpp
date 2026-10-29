#include <random>

#include "canfd/frame.hpp"
#include "testfw.hpp"

using namespace canfd;

namespace {
Frame make(uint32_t id, uint8_t len, uint64_t seed) {
  Frame f;
  f.id = id;
  f.len = len;
  std::mt19937_64 r(seed);
  for (int i = 0; i < len; ++i) f.data[static_cast<size_t>(i)] = static_cast<uint8_t>(r());
  return f;
}
}  // namespace

TEST(CanFd, DlcMapping) {
  CHECK_EQ(len_to_dlc(0), 0);
  CHECK_EQ(len_to_dlc(8), 8);
  CHECK_EQ(len_to_dlc(9), 9);
  CHECK_EQ(len_to_dlc(12), 9);
  CHECK_EQ(len_to_dlc(33), 14);
  CHECK_EQ(len_to_dlc(64), 15);
  for (uint8_t d = 0; d < 16; ++d) CHECK_EQ(len_to_dlc(dlc_to_len(d)), d);
  CHECK_EQ(round_up_len(13), 16);
  CHECK_EQ(round_up_len(49), 64);
}

TEST(CanFd, CrcLengthBySize) {
  CHECK_EQ(encode(make(0x100, 16, 1)).crc_len, 17);
  CHECK_EQ(encode(make(0x100, 20, 1)).crc_len, 21);
  CHECK_EQ(encode(make(0x100, 64, 1)).crc_len, 21);
}

TEST(CanFd, CleanFramePassesCrc) {
  for (uint8_t len : {0, 1, 8, 12, 16, 20, 32, 48, 64}) CHECK(crc_ok(encode(make(0x123, len, len))));
}

TEST(CanFd, AllSingleBitErrorsDetected) {
  const WireImage w0 = encode(make(0x2A5, 64, 7));
  int missed = 0;
  for (size_t i = 0; i < w0.bits.size(); ++i) {
    WireImage w = w0;
    w.bits[i] ^= 1U;
    if (crc_ok(w)) ++missed;
  }
  CHECK_EQ(missed, 0);
}

TEST(CanFd, AllDoubleBitErrorsDetectedCrc17) {
  const WireImage w0 = encode(make(0x055, 8, 3));
  int missed = 0;
  for (size_t i = 0; i < w0.bits.size(); ++i) {
    for (size_t j = i + 1; j < w0.bits.size(); ++j) {
      WireImage w = w0;
      w.bits[i] ^= 1U;
      w.bits[j] ^= 1U;
      if (crc_ok(w)) ++missed;
    }
  }
  CHECK_EQ(missed, 0);
}

TEST(CanFd, RandomBurstsDetected) {
  std::mt19937_64 r(5);
  const WireImage w0 = encode(make(0x321, 64, 9));
  int missed = 0;
  for (int trial = 0; trial < 20000; ++trial) {
    WireImage w = w0;
    const int k = 2 + static_cast<int>(r() % 4);  // 2..5 flipped bits (CRC-21 HD = 6)
    for (int i = 0; i < k; ++i) w.bits[r() % w.bits.size()] ^= 1U;
    if (w.bits != w0.bits && crc_ok(w)) ++missed;
  }
  CHECK_EQ(missed, 0);
}

TEST(CanFd, DecodeRoundTrip) {
  const Frame f = make(0x1AB, 48, 11);
  const Frame g = decode(encode(f), f);
  CHECK_EQ(g.id, f.id);
  for (int i = 0; i < 48; ++i) CHECK_EQ(g.data[static_cast<size_t>(i)], f.data[static_cast<size_t>(i)]);
}

TEST(CanFd, StuffBitsZeroPayload) {
  Frame f;
  f.id = 0;
  f.len = 64;  // all zero: one stuff bit every 5 bits in the data field
  const WireImage w = encode(f);
  CHECK(w.dyn_stuff_data >= 100);
  Frame g = make(0x555, 64, 1);
  CHECK(encode(g).dyn_stuff_data < w.dyn_stuff_data);
}

TEST(CanFd, AlternatingPatternNoStuffing) {
  Frame f;
  f.id = 0x2AA;
  f.len = 8;
  for (int i = 0; i < 8; ++i) f.data[static_cast<size_t>(i)] = 0x55;
  const WireImage w = encode(f);
  CHECK(w.dyn_stuff_data <= 1);
}

TEST(CanFd, DurationOf64ByteFrame) {
  const WireImage w = encode(make(0x100, 64, 2));
  const BitTiming bt;  // 500 k / 2 M
  const uint64_t ns = frame_duration_ns(w, true, bt);
  // ~29 nominal bits at 2 us + ~600 data bits at 0.5 us
  CHECK(ns > 300000 && ns < 400000);
  CHECK(frame_duration_ns(w, false, bt) > 3 * ns);
}

TEST(CanFd, DurationGrowsWithPayload) {
  const BitTiming bt;
  uint64_t prev = 0;
  for (uint8_t len : {0, 8, 16, 32, 64}) {
    const uint64_t d = frame_duration_ns(encode(make(0x100, len, 4)), true, bt);
    CHECK(d > prev);
    prev = d;
  }
}

TEST(CanFd, WireBitsIncludesOverhead) {
  const WireImage w = encode(make(0x100, 8, 2));
  CHECK(wire_bits(w) > w.bits.size() + 12);
}

TEST(CanFd, CrcKnownProperties) {
  std::vector<uint8_t> zero(40, 0);
  // register starts with a leading 1, so an all-zero message has a non-zero CRC
  CHECK(crc17(zero, 0, zero.size()) != 0u);
  CHECK(crc21(zero, 0, zero.size()) != 0u);
  CHECK(crc17(zero, 0, zero.size()) < (1u << 17));
  CHECK(crc21(zero, 0, zero.size()) < (1u << 21));
}

TEST_MAIN()
