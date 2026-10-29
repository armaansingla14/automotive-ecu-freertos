#include "ecu/network.hpp"
#include "sim/kernel.hpp"
#include "testfw.hpp"

using namespace ecu;

TEST(Network, TwentyNodesFaultFreeFullDelivery) {
  NetConfig c;
  c.seed = 5;
  Network net(c);
  net.run_until_ms(3000);
  const auto s = net.metrics().summarize(1000000, 2800000);
  CHECK(s.expected > 3000u);
  CHECK_EQ(s.in_deadline, s.expected);
  const double u = net.bus().utilization(3000000);
  CHECK(u > 0.30 && u < 0.45);
  CHECK_EQ(net.bus().error_frames, 0u);
}

TEST(Network, Deterministic) {
  uint64_t a, b;
  double la, lb;
  {
    NetConfig c;
    c.seed = 9;
    c.ber = 1e-5;
    c.background_load = 0.2;
    Network net(c);
    net.run_until_ms(2000);
    a = net.bus().error_frames;
    la = net.metrics().summarize(0, 2000000).pct(0.99);
  }
  {
    NetConfig c;
    c.seed = 9;
    c.ber = 1e-5;
    c.background_load = 0.2;
    Network net(c);
    net.run_until_ms(2000);
    b = net.bus().error_frames;
    lb = net.metrics().summarize(0, 2000000).pct(0.99);
  }
  CHECK_EQ(a, b);
  CHECK_NEAR(la, lb, 0.0);
}

TEST(Network, BackgroundLoadRaisesUtilization) {
  NetConfig c;
  c.seed = 2;
  c.tester = false;
  double base;
  {
    Network net(c);
    net.run_until_ms(2000);
    base = net.bus().utilization(2000000);
  }
  c.background_load = 0.3;
  Network net(c);
  net.run_until_ms(2000);
  const double u = net.bus().utilization(2000000);
  CHECK_NEAR(u - base, 0.3 * 1.9 / 2.0, 0.04);  // background starts at 0.1 s
}

TEST(Network, TesterGetsResponses) {
  NetConfig c;
  c.seed = 4;
  Network net(c);
  net.run_until_ms(3000);
  CHECK(net.tester().requests >= 25u);
  CHECK(net.tester().positive + 1 >= net.tester().requests);
  CHECK_EQ(net.tester().negative, 0u);
}

TEST(Network, ModerateBerStillDelivers) {
  NetConfig c;
  c.seed = 6;
  c.ber = 1e-5;
  Network net(c);
  net.run_until_ms(3000);
  const auto s = net.metrics().summarize(1000000, 2800000);
  CHECK(net.bus().error_frames > 0u);
  CHECK_EQ(s.in_deadline, s.expected);
  CHECK_EQ(net.bus().undetected_errors, 0u);
}

TEST(Network, DeadlineMonotonicIdsOrdered) {
  NetConfig c;
  Network net(c);
  // lower period -> lower identifier
  const auto& st = net.metrics().streams();
  for (const auto& a : st) {
    for (const auto& b : st) {
      if (a.name.rfind("sensor", 0) != 0 || b.name.rfind("sensor", 0) != 0) continue;
      if (a.period_ms < b.period_ms) CHECK(a.can_id < b.can_id);
    }
  }
}

TEST(Network, SaturatedBusTxCancelProtectsControlTraffic) {
  NetConfig c;
  c.seed = 11;
  c.background_load = 0.7;
  double with_cancel, without;
  {
    c.tx_cancel = true;
    Network net(c);
    net.run_until_ms(10000);
    with_cancel = net.metrics().summarize(1000000, 9800000).delivery_rate();
  }
  {
    c.tx_cancel = false;
    Network net(c);
    net.run_until_ms(10000);
    without = net.metrics().summarize(1000000, 9800000).delivery_rate();
  }
  CHECK(with_cancel > without);
}

TEST_MAIN()
