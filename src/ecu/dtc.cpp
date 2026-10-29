#include "ecu/dtc.hpp"

namespace ecu {

using namespace dtc_status;

DtcManager::DtcManager(int8_t threshold) : threshold_(threshold) {
  for (DtcId id : {DtcId::SensorRange, DtcId::SensorStuck, DtcId::SensorRate, DtcId::CommandTimeout,
                   DtcId::CanBusOff, DtcId::WatchdogReset}) {
    Entry e;
    e.id = id;
    entries_.push_back(e);
  }
}

DtcManager::Entry& DtcManager::get(DtcId id) {
  for (auto& e : entries_) {
    if (e.id == id) return e;
  }
  Entry e;
  e.id = id;
  entries_.push_back(e);
  return entries_.back();
}

const DtcManager::Entry* DtcManager::find(DtcId id) const {
  for (const auto& e : entries_) {
    if (e.id == id) return &e;
  }
  return nullptr;
}

uint8_t DtcManager::status(DtcId id) const {
  const Entry* e = find(id);
  return e ? e->status : 0;
}

void DtcManager::fail(Entry& e, const Snapshot& snap) {
  const bool was_failed = e.status & kTestFailed;
  e.status |= kTestFailed | kTestFailedThisCycle | kPending;
  if (!was_failed && e.occurrences < 255) ++e.occurrences;
  if (!(e.status & kConfirmed)) {
    e.status |= kConfirmed;
    // freeze frame on first confirmation only (oldest occurrence wins)
    if (!e.ff.valid) {
      e.ff.valid = true;
      e.ff.timestamp_ms = snap.now_ms;
      e.ff.odometer = cycles_;
      for (int i = 0; i < 3; ++i) e.ff.sensor[i] = snap.sensor[i];
      e.ff.supply_v = snap.supply_v;
    }
  }
  e.ff.occurrence = e.occurrences;
}

void DtcManager::report(DtcId id, bool failed, const Snapshot& snap) {
  Entry& e = get(id);
  if (failed) {
    if (e.counter < threshold_) ++e.counter;
    if (e.counter >= threshold_) fail(e, snap);
  } else {
    if (e.counter > -threshold_) --e.counter;
    if (e.counter <= -threshold_) e.status &= static_cast<uint8_t>(~kTestFailed);
  }
}

void DtcManager::report_event(DtcId id, const Snapshot& snap) {
  Entry& e = get(id);
  e.counter = threshold_;
  fail(e, snap);
  // an event is not a continuously monitored condition
  e.status &= static_cast<uint8_t>(~kTestFailed);
  e.counter = 0;
}

void DtcManager::clear_all() {
  for (auto& e : entries_) {
    e.status = 0;
    e.counter = 0;
    e.occurrences = 0;
    e.ff = FreezeFrame{};
  }
}

void DtcManager::new_operation_cycle() {
  ++cycles_;
  for (auto& e : entries_) {
    // pending is cleared after a cycle without failure
    if (!(e.status & kTestFailedThisCycle)) e.status &= static_cast<uint8_t>(~kPending);
    e.status &= static_cast<uint8_t>(~kTestFailedThisCycle);
  }
}

std::vector<const DtcManager::Entry*> DtcManager::by_status_mask(uint8_t mask) const {
  std::vector<const Entry*> out;
  for (const auto& e : entries_) {
    if (e.status & mask) out.push_back(&e);
  }
  return out;
}

}  // namespace ecu
