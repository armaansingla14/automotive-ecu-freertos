#include "ecu/uds.hpp"

#include <cstring>

namespace ecu {

namespace {
void put24(std::vector<uint8_t>& v, uint32_t x) {
  v.push_back(static_cast<uint8_t>(x >> 16));
  v.push_back(static_cast<uint8_t>(x >> 8));
  v.push_back(static_cast<uint8_t>(x));
}
void put16(std::vector<uint8_t>& v, uint16_t x) {
  v.push_back(static_cast<uint8_t>(x >> 8));
  v.push_back(static_cast<uint8_t>(x));
}
void put32(std::vector<uint8_t>& v, uint32_t x) {
  put16(v, static_cast<uint16_t>(x >> 16));
  put16(v, static_cast<uint16_t>(x));
}
void putf(std::vector<uint8_t>& v, float f) {
  uint32_t x;
  std::memcpy(&x, &f, 4);
  put32(v, x);
}
}  // namespace

std::vector<uint8_t> UdsServer::handle(const std::vector<uint8_t>& req) {
  using namespace uds;
  if (req.empty()) return {};
  const uint8_t sid = req[0];
  switch (sid) {
    case kTesterPresent: {
      if (req.size() != 2) return nrc(sid, kNrcIncorrectLength);
      if ((req[1] & 0x7F) != 0) return nrc(sid, kNrcSubFunctionNotSupported);
      if (req[1] & 0x80) return {};  // suppressPosRspMsgIndicationBit
      return {static_cast<uint8_t>(sid + 0x40), 0x00};
    }
    case kDiagSessionControl: {
      if (req.size() != 2) return nrc(sid, kNrcIncorrectLength);
      const uint8_t s = req[1] & 0x7F;
      if (s < 1 || s > 3) return nrc(sid, kNrcSubFunctionNotSupported);
      session_ = s;
      // P2 = 50 ms, P2* = 5000 ms (in 10 ms units)
      return {0x50, s, 0x00, 0x32, 0x01, 0xF4};
    }
    case kEcuReset: {
      if (req.size() != 2) return nrc(sid, kNrcIncorrectLength);
      if ((req[1] & 0x7F) != 0x01) return nrc(sid, kNrcSubFunctionNotSupported);
      reset_requested_ = true;
      return {0x51, 0x01};
    }
    case kClearDtc: {
      if (req.size() != 4) return nrc(sid, kNrcIncorrectLength);
      const uint32_t group = (uint32_t{req[1]} << 16) | (uint32_t{req[2]} << 8) | req[3];
      if (group != 0xFFFFFF) return nrc(sid, kNrcRequestOutOfRange);
      dtc_.clear_all();
      return {0x54};
    }
    case kReadDtc: {
      if (req.size() < 2) return nrc(sid, kNrcIncorrectLength);
      const uint8_t sub = req[1];
      if (sub == kReportNumberOfDtcByMask || sub == kReportDtcByStatusMask) {
        if (req.size() != 3) return nrc(sid, kNrcIncorrectLength);
        const auto list = dtc_.by_status_mask(req[2]);
        std::vector<uint8_t> r{0x59, sub, dtc_status::kAvailabilityMask};
        if (sub == kReportNumberOfDtcByMask) {
          r.push_back(0x01);  // ISO 14229-1 DTC format
          put16(r, static_cast<uint16_t>(list.size()));
        } else {
          for (const auto* e : list) {
            put24(r, static_cast<uint32_t>(e->id));
            r.push_back(e->status);
          }
        }
        return r;
      }
      if (sub == kReportSnapshotByDtc) {
        if (req.size() != 6) return nrc(sid, kNrcIncorrectLength);
        const uint32_t id = (uint32_t{req[2]} << 16) | (uint32_t{req[3]} << 8) | req[4];
        const auto* e = dtc_.find(static_cast<DtcId>(id));
        if (!e) return nrc(sid, kNrcRequestOutOfRange);
        std::vector<uint8_t> r{0x59, sub};
        put24(r, id);
        r.push_back(e->status);
        if (e->ff.valid && (req[5] == 0x01 || req[5] == 0xFF)) {
          r.push_back(0x01);  // record number
          r.push_back(6);     // number of identifiers
          put16(r, 0xDD00);
          put32(r, e->ff.timestamp_ms);
          put16(r, 0xDD01);
          put32(r, e->ff.odometer);
          put16(r, 0xDD02);
          putf(r, e->ff.sensor[0]);
          put16(r, 0xDD03);
          putf(r, e->ff.sensor[1]);
          put16(r, 0xDD04);
          putf(r, e->ff.supply_v);
          put16(r, 0xDD05);
          r.push_back(e->ff.occurrence);
        }
        return r;
      }
      return nrc(sid, kNrcSubFunctionNotSupported);
    }
    case kReadDid: {
      if (req.size() < 3 || (req.size() - 1) % 2 != 0) return nrc(sid, kNrcIncorrectLength);
      std::vector<uint8_t> r{0x62};
      for (size_t i = 1; i + 1 < req.size(); i += 2) {
        const uint16_t did = static_cast<uint16_t>((req[i] << 8) | req[i + 1]);
        std::vector<uint8_t> data;
        if (!dids_ || !dids_(did, data)) return nrc(sid, kNrcRequestOutOfRange);
        put16(r, did);
        r.insert(r.end(), data.begin(), data.end());
      }
      return r;
    }
    default:
      return nrc(sid, kNrcServiceNotSupported);
  }
}

}  // namespace ecu
