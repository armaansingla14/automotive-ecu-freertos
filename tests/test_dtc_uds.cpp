#include <cstring>

#include "ecu/dtc.hpp"
#include "ecu/uds.hpp"
#include "testfw.hpp"

using namespace ecu;
using namespace ecu::dtc_status;

namespace {
DtcManager::Snapshot snap(uint32_t t = 100) {
  DtcManager::Snapshot s;
  s.now_ms = t;
  s.sensor[0] = 1.5f;
  s.sensor[1] = 2.5f;
  s.sensor[2] = 3.5f;
  s.supply_v = 13.8f;
  return s;
}

UdsServer server(DtcManager& d) {
  return UdsServer(d, [](uint16_t did, std::vector<uint8_t>& out) {
    if (did == 0xF18C) {
      out.push_back(7);
      return true;
    }
    return false;
  });
}
}  // namespace

TEST(Dtc, DebounceBeforeConfirm) {
  DtcManager d(3);
  d.report(DtcId::SensorRange, true, snap());
  d.report(DtcId::SensorRange, true, snap());
  CHECK_EQ(d.status(DtcId::SensorRange), 0);
  d.report(DtcId::SensorRange, true, snap());
  const uint8_t s = d.status(DtcId::SensorRange);
  CHECK(s & kTestFailed);
  CHECK(s & kConfirmed);
  CHECK(s & kPending);
  CHECK(s & kTestFailedThisCycle);
}

TEST(Dtc, InterruptedFailuresDoNotConfirm) {
  DtcManager d(3);
  for (int i = 0; i < 10; ++i) {
    d.report(DtcId::SensorRate, true, snap());
    d.report(DtcId::SensorRate, false, snap());
  }
  CHECK_EQ(d.status(DtcId::SensorRate), 0);
}

TEST(Dtc, PassingClearsTestFailedKeepsConfirmed) {
  DtcManager d(3);
  for (int i = 0; i < 3; ++i) d.report(DtcId::SensorStuck, true, snap());
  for (int i = 0; i < 6; ++i) d.report(DtcId::SensorStuck, false, snap());
  const uint8_t s = d.status(DtcId::SensorStuck);
  CHECK(!(s & kTestFailed));
  CHECK(s & kConfirmed);
}

TEST(Dtc, FreezeFrameOnFirstConfirmation) {
  DtcManager d(1);
  d.report(DtcId::SensorRange, true, snap(1234));
  for (int i = 0; i < 3; ++i) d.report(DtcId::SensorRange, false, snap());
  d.report(DtcId::SensorRange, true, snap(9999));  // counter -1 -> 0
  d.report(DtcId::SensorRange, true, snap(9999));  // 0 -> +1: failed again
  const auto* e = d.find(DtcId::SensorRange);
  REQUIRE(e != nullptr);
  CHECK(e->ff.valid);
  CHECK_EQ(e->ff.timestamp_ms, 1234u);  // first occurrence kept
  CHECK_NEAR(e->ff.sensor[1], 2.5, 1e-6);
  CHECK_EQ(e->occurrences, 2);
}

TEST(Dtc, OperationCycleAgesPending) {
  DtcManager d(1);
  d.report(DtcId::CanBusOff, true, snap());
  d.report(DtcId::CanBusOff, false, snap());
  d.new_operation_cycle();
  CHECK(d.status(DtcId::CanBusOff) & kPending);  // failed during the ended cycle
  d.new_operation_cycle();
  CHECK(!(d.status(DtcId::CanBusOff) & kPending));
  CHECK(d.status(DtcId::CanBusOff) & kConfirmed);
}

TEST(Dtc, EventIsConfirmedImmediately) {
  DtcManager d(3);
  d.report_event(DtcId::WatchdogReset, snap());
  CHECK(d.status(DtcId::WatchdogReset) & kConfirmed);
  CHECK(!(d.status(DtcId::WatchdogReset) & kTestFailed));
}

TEST(Dtc, ClearAll) {
  DtcManager d(1);
  d.report(DtcId::SensorRange, true, snap());
  d.clear_all();
  CHECK_EQ(d.status(DtcId::SensorRange), 0);
  CHECK(!d.find(DtcId::SensorRange)->ff.valid);
}

TEST(Uds, ReadDtcByStatusMask) {
  DtcManager d(1);
  d.report(DtcId::SensorRange, true, snap());
  d.report(DtcId::CommandTimeout, true, snap());
  auto s = server(d);
  const auto r = s.handle({0x19, 0x02, kConfirmed});
  REQUIRE(r.size() == 3 + 2 * 4);
  CHECK_EQ(r[0], 0x59);
  CHECK_EQ(r[1], 0x02);
  CHECK_EQ(r[3], 0x01);  // 0x012300
  CHECK_EQ(r[4], 0x23);
  CHECK(r[6] & kConfirmed);
}

TEST(Uds, ReportNumberOfDtc) {
  DtcManager d(1);
  d.report(DtcId::SensorRange, true, snap());
  auto s = server(d);
  const auto r = s.handle({0x19, 0x01, kConfirmed});
  REQUIRE(r.size() == 6);
  CHECK_EQ(r[5], 1);
}

TEST(Uds, ReadSnapshotRecord) {
  DtcManager d(1);
  d.report(DtcId::SensorRange, true, snap(4321));
  auto s = server(d);
  const auto r = s.handle({0x19, 0x04, 0x01, 0x23, 0x00, 0x01});
  REQUIRE(r.size() > 10);
  CHECK_EQ(r[0], 0x59);
  CHECK_EQ(r[6], 0x01);  // record number
  CHECK_EQ(r[7], 6);     // identifiers
  CHECK_EQ(r[8], 0xDD);
  const uint32_t ts = (uint32_t{r[10]} << 24) | (uint32_t{r[11]} << 16) | (uint32_t{r[12]} << 8) | r[13];
  CHECK_EQ(ts, 4321u);
}

TEST(Uds, SnapshotUnknownDtc) {
  DtcManager d(1);
  auto s = server(d);
  const auto r = s.handle({0x19, 0x04, 0x12, 0x34, 0x56, 0x01});
  REQUIRE(r.size() == 3);
  CHECK_EQ(r[0], 0x7F);
  CHECK_EQ(r[2], uds::kNrcRequestOutOfRange);
}

TEST(Uds, ClearDtc) {
  DtcManager d(1);
  d.report(DtcId::SensorRange, true, snap());
  auto s = server(d);
  const auto r = s.handle({0x14, 0xFF, 0xFF, 0xFF});
  REQUIRE(r.size() == 1);
  CHECK_EQ(r[0], 0x54);
  CHECK_EQ(d.status(DtcId::SensorRange), 0);
  const auto bad = s.handle({0x14, 0x01, 0x00, 0x00});
  CHECK_EQ(bad[2], uds::kNrcRequestOutOfRange);
}

TEST(Uds, ReadDid) {
  DtcManager d;
  auto s = server(d);
  const auto r = s.handle({0x22, 0xF1, 0x8C});
  REQUIRE(r.size() == 4);
  CHECK_EQ(r[0], 0x62);
  CHECK_EQ(r[3], 7);
  const auto bad = s.handle({0x22, 0x12, 0x34});
  CHECK_EQ(bad[0], 0x7F);
  CHECK_EQ(bad[2], uds::kNrcRequestOutOfRange);
}

TEST(Uds, TesterPresentAndSuppress) {
  DtcManager d;
  auto s = server(d);
  const auto r = s.handle({0x3E, 0x00});
  REQUIRE(r.size() == 2);
  CHECK_EQ(r[0], 0x7E);
  CHECK(s.handle({0x3E, 0x80}).empty());
}

TEST(Uds, NegativeResponses) {
  DtcManager d;
  auto s = server(d);
  CHECK_EQ(s.handle({0x31, 0x01})[2], uds::kNrcServiceNotSupported);
  CHECK_EQ(s.handle({0x19})[2], uds::kNrcIncorrectLength);
  CHECK_EQ(s.handle({0x19, 0x0A})[2], uds::kNrcSubFunctionNotSupported);
  CHECK_EQ(s.handle({0x22, 0xF1})[2], uds::kNrcIncorrectLength);
  CHECK_EQ(s.handle({0x10, 0x09})[2], uds::kNrcSubFunctionNotSupported);
}

TEST(Uds, SessionAndReset) {
  DtcManager d;
  auto s = server(d);
  const auto r = s.handle({0x10, 0x03});
  REQUIRE(r.size() == 6);
  CHECK_EQ(s.session(), 3);
  CHECK(!s.reset_requested());
  CHECK_EQ(s.handle({0x11, 0x01})[0], 0x51);
  CHECK(s.reset_requested());
  s.on_boot();
  CHECK(!s.reset_requested());
  CHECK_EQ(s.session(), 1);
}

TEST_MAIN()
