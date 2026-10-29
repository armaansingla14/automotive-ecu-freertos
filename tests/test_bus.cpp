#include <vector>

#include "canfd/bus.hpp"
#include "sim/kernel.hpp"
#include "testfw.hpp"

using namespace canfd;

namespace {
struct Rx {
  uint32_t id;
  uint64_t t;
};

Frame frame(uint32_t id, uint8_t len = 8) {
  Frame f;
  f.id = id;
  f.len = len;
  for (int i = 0; i < len; ++i) f.data[static_cast<size_t>(i)] = static_cast<uint8_t>(i * 37 + id);
  return f;
}

sim::Kernel& fresh() {
  sim::Kernel& k = sim::Kernel::get();
  k.clear();
  return k;
}
}  // namespace

TEST(Bus, ArbitrationLowestIdWins) {
  sim::Kernel& k = fresh();
  Bus bus(k, BusConfig{});
  Controller& a = bus.attach(0);
  Controller& b = bus.attach(1);
  Controller& c = bus.attach(2);
  Controller& rx = bus.attach(3);
  std::vector<Rx> got;
  rx.rx_isr = [&](const Frame& f, uint64_t t) { got.push_back({f.id, t}); };
  a.submit(frame(0x300), 0);
  b.submit(frame(0x100), 0);
  c.submit(frame(0x200), 0);
  k.run_until(10000);
  REQUIRE(got.size() == 3);
  CHECK_EQ(got[0].id, 0x100u);
  CHECK_EQ(got[1].id, 0x200u);
  CHECK_EQ(got[2].id, 0x300u);
  CHECK(bus.arbitration_losses >= 3u);
}

TEST(Bus, ControllerSendsOwnBuffersInIdOrder) {
  sim::Kernel& k = fresh();
  Bus bus(k, BusConfig{});
  Controller& a = bus.attach(0);
  Controller& rx = bus.attach(1);
  std::vector<uint32_t> ids;
  rx.rx_isr = [&](const Frame& f, uint64_t) { ids.push_back(f.id); };
  a.submit(frame(0x250), 0);
  a.submit(frame(0x150), 0);
  a.submit(frame(0x050), 0);
  k.run_until(10000);
  REQUIRE(ids.size() == 3);
  CHECK_EQ(ids[0], 0x050u);
  CHECK_EQ(ids[2], 0x250u);
}

TEST(Bus, DeliveryTimeMatchesFrameDuration) {
  sim::Kernel& k = fresh();
  Bus bus(k, BusConfig{});
  Controller& a = bus.attach(0);
  Controller& rx = bus.attach(1);
  uint64_t t_rx = 0;
  rx.rx_isr = [&](const Frame&, uint64_t t) { t_rx = t; };
  const Frame f = frame(0x123, 64);
  a.submit(f, 0);
  k.run_until(10000);
  const uint64_t expect_us = (frame_duration_ns(encode(f), true, BitTiming{}) + 999) / 1000;
  CHECK_EQ(t_rx, expect_us);
}

TEST(Bus, AcceptanceFilter) {
  sim::Kernel& k = fresh();
  Bus bus(k, BusConfig{});
  Controller& a = bus.attach(0);
  Controller& rx = bus.attach(1);
  rx.add_filter(0x200, 0x7F0);  // 0x200..0x20F
  int n = 0;
  rx.rx_isr = [&](const Frame&, uint64_t) { ++n; };
  a.submit(frame(0x205), 0);
  a.submit(frame(0x210), 0);
  a.submit(frame(0x1FF), 0);
  k.run_until(10000);
  CHECK_EQ(n, 1);
  CHECK(rx.accepts(0x20F));
  CHECK(!rx.accepts(0x300));
}

TEST(Bus, TxBufferLimit) {
  sim::Kernel& k = fresh();
  Bus bus(k, BusConfig{});
  Controller& a = bus.attach(0);
  CHECK(a.submit(frame(1), 0));
  CHECK(a.submit(frame(2), 0));
  CHECK(a.submit(frame(3), 0));
  CHECK(!a.submit(frame(4), 0));
  CHECK_EQ(a.free_buffers(), 0u);
}

TEST(Bus, CancelLowerPriority) {
  sim::Kernel& k = fresh();
  Bus bus(k, BusConfig{});
  Controller& a = bus.attach(0);
  a.submit(frame(0x700), 0);
  a.submit(frame(0x100), 0);
  a.submit(frame(0x200), 0);
  CHECK(!a.cancel_lower_priority(0x750));
  CHECK(a.cancel_lower_priority(0x050));
  CHECK_EQ(a.tx_cancelled, 1u);
  CHECK(a.submit(frame(0x050), 0));
}

TEST(Bus, ErrorFramesAndRetransmission) {
  sim::Kernel& k = fresh();
  BusConfig cfg;
  cfg.seed = 3;
  Bus bus(k, cfg);
  Controller& a = bus.attach(0);
  Controller& rx = bus.attach(1);
  int n = 0;
  rx.rx_isr = [&](const Frame&, uint64_t) { ++n; };
  bus.add_noise_window(0, 2000, 0.5);  // nothing gets through for 2 ms
  a.submit(frame(0x100, 64), 0);
  k.run_until(20000);
  CHECK_EQ(n, 1);
  CHECK(bus.error_frames > 0u);
  CHECK_EQ(a.tx_ok, 1u);
  CHECK(a.tx_errors > 0u);
}

TEST(Bus, FaultConfinementToBusOffAndRecovery) {
  sim::Kernel& k = fresh();
  Bus bus(k, BusConfig{});
  Controller& bad = bus.attach(0);
  Controller& rx = bus.attach(1);
  (void)rx;
  bad.tx_fault = true;
  bad.submit(frame(0x100, 64), 0);
  // each error adds 8 to TEC: error passive after 16, bus-off after 32
  k.run_until(50000);
  CHECK(bad.bus_off_events >= 1u);
  CHECK(bad.state() == ErrState::Active || bad.state() == ErrState::BusOff);
  CHECK_EQ(bad.free_buffers(), 3u);  // pending frames dropped at bus-off
  bad.tx_fault = false;
  k.run_until(60000);  // 128 x 11 bits at 500 kbit/s = 2.8 ms
  CHECK(bad.state() == ErrState::Active);
  CHECK(bad.submit(frame(0x100), k.now_us()));
}

TEST(Bus, ErrorPassiveThreshold) {
  sim::Kernel& k = fresh();
  Bus bus(k, BusConfig{});
  Controller& bad = bus.attach(0);
  bus.attach(1);
  bad.tx_fault = true;
  bad.submit(frame(0x100, 64), 0);
  bool saw_passive = false;
  for (uint64_t t = 100; t < 20000 && bad.bus_off_events == 0; t += 100) {
    k.run_until(t);
    if (bad.state() == ErrState::Passive) {
      saw_passive = true;
      CHECK(bad.tec() >= 128);
    }
  }
  CHECK(saw_passive);
}

TEST(Bus, HealthyNodesUnaffectedByBusOffNode) {
  sim::Kernel& k = fresh();
  Bus bus(k, BusConfig{});
  Controller& bad = bus.attach(0);
  Controller& good = bus.attach(1);
  Controller& rx = bus.attach(2);
  int n = 0;
  rx.rx_isr = [&](const Frame& f, uint64_t) {
    if (f.id == 0x200) ++n;
  };
  bad.tx_fault = true;
  for (int i = 0; i < 50; ++i) {
    k.at(static_cast<uint64_t>(i) * 1000, [&]() {
      bad.submit(frame(0x100, 64), k.now_us());
      good.submit(frame(0x200, 64), k.now_us());
    });
  }
  k.run_until(100000);
  // the babbling node delays others until it reaches bus-off; after that
  // the healthy node's traffic flows (some early frames are lost because
  // its TX buffers were full)
  CHECK(n >= 35);
  CHECK(bad.bus_off_events >= 1u);
  CHECK(good.tx_ok == static_cast<uint64_t>(n));
}

TEST(Bus, UtilizationAccounting) {
  sim::Kernel& k = fresh();
  Bus bus(k, BusConfig{});
  Controller& a = bus.attach(0);
  bus.attach(1);
  const Frame f = frame(0x100, 64);
  const uint64_t d = frame_duration_ns(encode(f), true, BitTiming{});
  for (int i = 0; i < 10; ++i) k.at(static_cast<uint64_t>(i) * 1000, [&]() { a.submit(f, k.now_us()); });
  k.run_until(10000);
  CHECK_NEAR(bus.utilization(10000), 10.0 * static_cast<double>(d) / 1e7, 1e-6);
}

TEST(Bus, LowBerAllDetectedNoUndetected) {
  sim::Kernel& k = fresh();
  BusConfig cfg;
  cfg.ber = 1e-4;
  cfg.seed = 9;
  Bus bus(k, cfg);
  Controller& a = bus.attach(0);
  Controller& rx = bus.attach(1);
  int n = 0;
  rx.rx_isr = [&](const Frame&, uint64_t) { ++n; };
  for (int i = 0; i < 2000; ++i) k.at(static_cast<uint64_t>(i) * 500, [&]() { a.submit(frame(0x100, 64), k.now_us()); });
  k.run_until(2000000);
  CHECK_EQ(n, 2000);
  CHECK(bus.error_frames > 50u);
  CHECK_EQ(bus.undetected_errors, 0u);
}

TEST(Bus, DisabledControllerNeitherSendsNorReceives) {
  sim::Kernel& k = fresh();
  Bus bus(k, BusConfig{});
  Controller& a = bus.attach(0);
  Controller& rx = bus.attach(1);
  int n = 0;
  rx.rx_isr = [&](const Frame&, uint64_t) { ++n; };
  rx.enabled = false;
  a.submit(frame(0x100), 0);
  k.run_until(5000);
  CHECK_EQ(n, 0);
  a.enabled = false;
  CHECK(!a.submit(frame(0x101), k.now_us()));
}

TEST_MAIN()
