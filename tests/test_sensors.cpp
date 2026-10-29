#include "ecu/messages.hpp"
#include "ecu/sensors.hpp"
#include "testfw.hpp"

using namespace ecu;

namespace {
const Plausibility::Limits kLim = {0.25f, 4.75f, 0.8f, 25};
}

TEST(Sensors, NominalSignalsPlausible) {
  SensorModel m(1, 0.3f);
  Plausibility p[3] = {Plausibility(kLim), Plausibility(kLim), Plausibility(kLim)};
  int fails = 0;
  for (uint32_t t = 0; t < 60000; t += 10) {
    float v[3];
    m.sample(t, v);
    for (int i = 0; i < 3; ++i) {
      const auto r = p[i].check(v[i]);
      fails += r.range_fail + r.rate_fail + r.stuck_fail;
    }
  }
  CHECK_EQ(fails, 0);
}

TEST(Sensors, OpenCircuitOutOfRange) {
  SensorModel m(2);
  m.set_fault(1, SensorModel::Fault::OpenCircuit);
  Plausibility p(kLim);
  float v[3];
  m.sample(100, v);
  CHECK(p.check(v[1]).range_fail);
}

TEST(Sensors, StuckDetectedAfterLimit) {
  SensorModel m(3);
  m.set_fault(0, SensorModel::Fault::Stuck);
  Plausibility p(kLim);
  int first = -1;
  for (int i = 0; i < 40; ++i) {
    float v[3];
    m.sample(static_cast<uint32_t>(i * 10), v);
    if (p.check(v[0]).stuck_fail && first < 0) first = i;
  }
  CHECK_EQ(first, 25);
}

TEST(Sensors, SpikesTriggerRateCheck) {
  SensorModel m(4);
  m.set_fault(2, SensorModel::Fault::Spike);
  Plausibility p(kLim);
  int rate = 0;
  for (int i = 0; i < 500; ++i) {
    float v[3];
    m.sample(static_cast<uint32_t>(i * 10), v);
    rate += p.check(v[2]).rate_fail;
  }
  CHECK(rate > 20);
}

TEST(Sensors, FaultRemovalRestoresSignal) {
  SensorModel m(5);
  m.set_fault(0, SensorModel::Fault::OpenCircuit);
  float v[3];
  m.sample(0, v);
  m.set_fault(0, SensorModel::Fault::None);
  m.sample(10, v);
  CHECK(v[0] < 4.0f);
}

TEST(Actuator, RateLimited) {
  Actuator a;
  a.update(0);
  a.command(50.f, 0);
  a.update(10);
  CHECK_NEAR(a.output(), 5.0, 1e-4);  // 0.5 per ms
  a.command(50.f, 190);
  a.update(200);
  CHECK_NEAR(a.output(), 50.0, 1e-4);
}

TEST(Actuator, TimeoutRampsToSafeState) {
  Actuator a;
  a.update(0);
  a.command(40.f, 0);
  for (uint32_t t = 10; t <= 140; t += 10) {
    a.command(40.f, t);
    a.update(t);
  }
  CHECK_NEAR(a.output(), 40.0, 1e-4);
  for (uint32_t t = 150; t <= 400; t += 10) a.update(t);
  CHECK(a.timed_out());
  CHECK_NEAR(a.output(), 0.0, 1e-4);
  CHECK_EQ(a.timeouts(), 1u);
  a.command(10.f, 410);
  CHECK(!a.timed_out());
}

TEST(Actuator, CommandClamped) {
  Actuator a;
  a.update(0);
  a.command(500.f, 990);
  a.update(1000);
  CHECK_NEAR(a.output(), 100.0, 1e-4);
}

TEST(Messages, SensorRoundTrip) {
  SensorMsg m{77, 123456789ULL, {1.f, 2.5f, -3.f}, 5};
  const auto f = pack(0x101, m);
  CHECK_EQ(f.len, 64);
  SensorMsg o{};
  REQUIRE(unpack(f, o));
  CHECK_EQ(o.seq, 77u);
  CHECK_EQ(o.t_us, 123456789ULL);
  CHECK_NEAR(o.value[1], 2.5, 1e-6);
  CHECK_EQ(o.flags, 5);
}

TEST(Messages, CommandRoundTrip) {
  CommandMsg c{9, 42, 12.34f};
  const auto f = pack(0x081, c);
  CHECK_EQ(f.len, 16);
  CommandMsg o{};
  REQUIRE(unpack(f, o));
  CHECK_NEAR(o.setpoint, 12.34, 0.01);
}

TEST(Messages, E2eCrcDetectsCorruption) {
  SensorMsg m{1, 2, {1, 2, 3}, 0};
  auto f = pack(0x100, m);
  f.data[13] ^= 0x10;
  SensorMsg o{};
  CHECK(!unpack(f, o));
  CommandMsg c{1, 2, 3};
  auto g = pack(0x080, c);
  g.len = 64;  // wrong length for a command
  CHECK(!unpack(g, c));
}

TEST(Messages, DiagFraming) {
  const uint8_t req[] = {0x19, 0x02, 0xFF};
  const auto f = pack_diag(0x705, req, sizeof req);
  CHECK_EQ(f.len, 4);
  uint8_t out[64];
  CHECK_EQ(unpack_diag(f, out, sizeof out), 3u);
  CHECK_EQ(out[0], 0x19);
  uint8_t big[70] = {0x22};
  const auto g = pack_diag(0x705, big, sizeof big);
  CHECK_EQ(g.len, 64);
  CHECK_EQ(g.data[0], 62);
}

TEST(Messages, Crc16KnownValue) {
  const uint8_t s[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  CHECK_EQ(crc16_ccitt(s, 9), 0x29B1);  // CRC-16/CCITT-FALSE check value
}

TEST_MAIN()
