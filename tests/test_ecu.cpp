// Firmware-level tests on a small network (6 ECUs) built with the same code
// as the 20-node experiments.
#include <vector>

#include "ecu/messages.hpp"
#include "ecu/network.hpp"
#include "sim/kernel.hpp"
#include "testfw.hpp"

using namespace ecu;

namespace {
NetConfig small(WdgMode mode = WdgMode::Supervised) {
  NetConfig c;
  c.nodes = 6;
  c.wdg = mode;
  c.tester = false;
  c.seed = 3;
  return c;
}

// first delivery of node's sensor stream after time t (us); -1 if none
int64_t first_rx_after(Network& net, int node, uint64_t t) {
  int64_t best = -1;
  for (const auto& kv : net.metrics().streams()[static_cast<size_t>(2 * node)].received) {
    if (kv.second > t && (best < 0 || static_cast<int64_t>(kv.second) < best)) best = static_cast<int64_t>(kv.second);
  }
  return best;
}

// diagnostic client on its own controller
struct Tester {
  canfd::Controller* c;
  std::vector<std::vector<uint8_t>> rsp;
  explicit Tester(Network& net) : c(&net.bus().attach(99)) {
    c->add_filter(0x740, 0x7C0);
    c->rx_isr = [this](const canfd::Frame& f, uint64_t) {
      uint8_t b[64];
      const size_t n = unpack_diag(f, b, sizeof b);
      rsp.emplace_back(b, b + n);
    };
  }
  void send(int node, std::vector<uint8_t> req) {
    c->submit(pack_diag(0x700u + static_cast<uint32_t>(node), req.data(), req.size()), sim::Kernel::get().now_us());
  }
};
}  // namespace

TEST(Ecu, AllStreamsDeliveredFaultFree) {
  Network net(small());
  net.run_until_ms(3000);
  const auto s = net.metrics().summarize(500000, 2800000);
  CHECK(s.expected > 1000u);
  CHECK_EQ(s.in_deadline, s.expected);
  for (int i = 0; i < 6; ++i) CHECK_EQ(sim::Kernel::get().node(i)->resets, 0u);
}

TEST(Ecu, SupervisedWatchdogRecoversRunawaySensorTask) {
  Network net(small());
  net.schedule_hang(1000, 2, EcuFirmware::kSensor);
  net.run_until_ms(3000);
  const sim::Node* n = sim::Kernel::get().node(2);
  REQUIRE(n->resets == 1u);
  const double detect_ms = static_cast<double>(n->reset_times_us[0]) / 1000.0 - 1000.0;
  CHECK(detect_ms > 0 && detect_ms < 400);
  CHECK(first_rx_after(net, 2, n->reset_times_us[0]) > 0);
  CHECK(net.ecu(2).nvm().dtc.status(DtcId::WatchdogReset) & dtc_status::kConfirmed);
  // the supervisor blames the first task that misses its deadline: the hung
  // sensor task itself or a lower-priority task it starves
  const int culprit = net.ecu(2).nvm().last_culprit;
  CHECK(culprit >= static_cast<int>(EcuFirmware::kSensor));
}

TEST(Ecu, NoWatchdogNoRecovery) {
  Network net(small(WdgMode::Off));
  net.schedule_hang(1000, 2, EcuFirmware::kSensor);
  net.run_until_ms(3000);
  CHECK_EQ(sim::Kernel::get().node(2)->resets, 0u);
  CHECK(first_rx_after(net, 2, 1100000) < 0);
}

TEST(Ecu, UnconditionalKickMissesLowPriorityHang) {
  Network net(small(WdgMode::Unconditional));
  net.schedule_hang(1000, 2, EcuFirmware::kSensor);
  net.run_until_ms(3000);
  CHECK_EQ(sim::Kernel::get().node(2)->resets, 0u);  // watchdog task still runs and kicks
}

TEST(Ecu, UnconditionalKickCatchesHighestPriorityHang) {
  Network net(small(WdgMode::Unconditional));
  net.schedule_hang(1000, 2, EcuFirmware::kWdg);
  net.run_until_ms(3000);
  CHECK(sim::Kernel::get().node(2)->resets >= 1u);
}

TEST(Ecu, DeadlockDetected) {
  Network net(small());
  net.schedule_deadlock(1000, 4, EcuFirmware::kRx);
  net.run_until_ms(3000);
  CHECK_EQ(sim::Kernel::get().node(4)->resets, 1u);
  CHECK_EQ(net.ecu(4).nvm().last_culprit, static_cast<int>(EcuFirmware::kRx));
}

TEST(Ecu, FastSupervisionResetsSooner) {
  uint64_t t_slow, t_fast;
  {
    Network net(small(WdgMode::Supervised));
    net.schedule_hang(1000, 1, EcuFirmware::kControl);
    net.run_until_ms(2000);
    REQUIRE(sim::Kernel::get().node(1)->resets == 1u);
    t_slow = sim::Kernel::get().node(1)->reset_times_us[0];
  }
  {
    Network net(small(WdgMode::SupervisedFast));
    net.schedule_hang(1000, 1, EcuFirmware::kControl);
    net.run_until_ms(2000);
    REQUIRE(sim::Kernel::get().node(1)->resets == 1u);
    t_fast = sim::Kernel::get().node(1)->reset_times_us[0];
  }
  CHECK(t_fast + 50000 <= t_slow);
}

TEST(Ecu, SequenceContinuesAcrossReset) {
  Network net(small());
  net.schedule_hang(1000, 3, EcuFirmware::kTx);
  net.run_until_ms(3000);
  const auto& prod = net.metrics().streams()[6].produced;  // sensor stream of node 3
  REQUIRE(prod.size() > 10);
  for (size_t i = 1; i < prod.size(); ++i) CHECK(prod[i].first == prod[i - 1].first + 1);
}

TEST(Ecu, CommandTimeoutWhenSenderDown) {
  Network net(small(WdgMode::Off));
  net.schedule_hang(1000, 0, EcuFirmware::kRx);  // node 0 (same prio as TX) stops transmitting
  net.run_until_ms(2000);
  // node 1 consumes node 0's commands
  CHECK(net.ecu(1).actuator().timed_out());
  CHECK(net.ecu(1).nvm().dtc.status(DtcId::CommandTimeout) & dtc_status::kConfirmed);
}

TEST(Ecu, UdsReadDtcAfterSensorFault) {
  Network net(small());
  Tester t(net);
  net.ecu(5).sensors().set_fault(0, SensorModel::Fault::OpenCircuit);
  net.run_until_ms(1000);
  t.send(5, {0x19, 0x02, 0x08});
  net.run_until_ms(1100);
  REQUIRE(t.rsp.size() == 1u);
  const auto& r = t.rsp[0];
  REQUIRE(r.size() >= 7u);
  CHECK_EQ(r[0], 0x59);
  CHECK_EQ(r[3], 0x01);
  CHECK_EQ(r[4], 0x23);  // P0123-like range DTC
}

TEST(Ecu, UdsClearDtc) {
  Network net(small());
  Tester t(net);
  net.ecu(5).sensors().set_fault(0, SensorModel::Fault::OpenCircuit);
  net.run_until_ms(500);
  net.ecu(5).sensors().set_fault(0, SensorModel::Fault::None);
  net.run_until_ms(1000);
  t.send(5, {0x14, 0xFF, 0xFF, 0xFF});
  net.run_until_ms(1100);
  REQUIRE(!t.rsp.empty());
  CHECK_EQ(t.rsp[0][0], 0x54);
  CHECK_EQ(net.ecu(5).nvm().dtc.status(DtcId::SensorRange), 0);
}

TEST(Ecu, UdsEcuResetReboots) {
  Network net(small());
  Tester t(net);
  net.run_until_ms(500);
  t.send(2, {0x11, 0x01});
  net.run_until_ms(1000);
  REQUIRE(!t.rsp.empty());
  CHECK_EQ(t.rsp[0][0], 0x51);
  const sim::Node* n = sim::Kernel::get().node(2);
  CHECK_EQ(n->resets, 1u);
  CHECK(n->reset_cause == OSAL_RESET_SOFTWARE);
  t.send(2, {0x22, 0x01, 0x04});  // reset cause DID
  net.run_until_ms(1100);
  REQUIRE(t.rsp.size() == 2u);
  CHECK_EQ(t.rsp[1][3], static_cast<uint8_t>(OSAL_RESET_SOFTWARE));
}

TEST(Ecu, UdsFreezeFrameOfWatchdogReset) {
  Network net(small());
  Tester t(net);
  net.schedule_hang(500, 1, EcuFirmware::kDiag);
  net.run_until_ms(1500);
  t.send(1, {0x19, 0x04, 0xD0, 0x00, 0x00, 0x01});
  net.run_until_ms(1700);
  REQUIRE(!t.rsp.empty());
  CHECK_EQ(t.rsp[0][0], 0x59);
  CHECK(t.rsp[0].size() > 8u);  // snapshot record present
}

TEST(Ecu, ResetEcuMissesFramesThenResumes) {
  Network net(small());
  net.schedule_hang(1000, 2, EcuFirmware::kRx);
  net.run_until_ms(3000);
  // node 2 consumes node 1's sensor stream; some instances are lost during the outage
  const auto s = net.metrics().summarize(1000000, 2800000, 0, 1);
  CHECK(s.in_deadline < s.expected);
  CHECK(s.in_deadline > s.expected * 8 / 10);
}

TEST_MAIN()
