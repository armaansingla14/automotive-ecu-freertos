#include "ecu/ecu_node.hpp"

#include <cstring>

#include "ecu/messages.hpp"

namespace ecu {

const char* to_string(WdgMode m) {
  switch (m) {
    case WdgMode::Off: return "off";
    case WdgMode::Unconditional: return "unconditional";
    case WdgMode::Supervised: return "supervised";
    case WdgMode::SupervisedFast: return "supervised_fast";
  }
  return "?";
}

namespace {
const Plausibility::Limits kLimits = {0.25f, 4.75f, 0.8f, 25};
}

EcuFirmware::EcuFirmware(const EcuConfig& cfg, canfd::Controller& can, Metrics* metrics)
    : cfg_(cfg),
      can_(can),
      metrics_(metrics),
      sensors_(cfg.seed, static_cast<float>(cfg.node) * 0.37f),
      plaus_{Plausibility(kLimits), Plausibility(kLimits), Plausibility(kLimits)},
      uds_(nvm_.dtc, [this](uint16_t did, std::vector<uint8_t>& out) { return read_did(did, out); }) {
  // acceptance filters: only what this ECU consumes
  for (uint32_t id : cfg_.rx_sensor_ids) can_.add_filter(id, 0x7FF);
  if (cfg_.rx_cmd_id) can_.add_filter(cfg_.rx_cmd_id, 0x7FF);
  can_.add_filter(cfg_.diag_req_id, 0x7FF);
}

void EcuFirmware::on_reset() {
  can_.enabled = false;
  can_.flush();
  can_.rx_isr = nullptr;
}

DtcManager::Snapshot EcuFirmware::snapshot() {
  DtcManager::Snapshot s;
  s.now_ms = osal_tick_ms() - boot_ms_;
  for (int i = 0; i < 3; ++i) s.sensor[i] = local_[i];
  s.supply_v = sensors_.supply_voltage();
  return s;
}

void EcuFirmware::boot() {
  boot_us_ = osal_time_us();
  boot_ms_ = osal_tick_ms();
  ++nvm_.boots;
  nvm_.dtc.new_operation_cycle();
  std::memset(hang_, 0, sizeof hang_);
  std::memset(deadlock_, 0, sizeof deadlock_);
  act_ = Actuator{};
  uds_.on_boot();
  for (auto& pl : plaus_) pl = Plausibility(kLimits);
  if (osal_reset_cause() == OSAL_RESET_WATCHDOG) nvm_.dtc.report_event(DtcId::WatchdogReset, snapshot());

  tx_q_ = osal_queue_create(8, sizeof(TxItem));
  rx_q_ = osal_queue_create(16, sizeof(canfd::Frame));
  act_q_ = osal_queue_create(4, sizeof(float));
  diag_q_ = osal_queue_create(2, sizeof(canfd::Frame));
  dead_q_ = osal_queue_create(1, 1);

  const uint32_t now = osal_tick_ms();
  for (int i = 0; i < kNumTasks; ++i) hb_[i] = now;
  hb_limit_[kWdg] = 0;
  hb_limit_[kRx] = 100;
  hb_limit_[kTx] = 100;
  hb_limit_[kSensor] = 3 * cfg_.sensor_period_ms + 20;
  hb_limit_[kControl] = 3 * cfg_.cmd_period_ms;
  hb_limit_[kActuator] = 100;
  hb_limit_[kDiag] = 200;

  osal_task_create("wdg", &EcuFirmware::wdg_task, this, 1024, 6);
  osal_task_create("can_rx", &EcuFirmware::rx_task, this, 2048, 5);
  osal_task_create("can_tx", &EcuFirmware::tx_task, this, 2048, 5);
  osal_task_create("sensor", &EcuFirmware::sensor_task, this, 2048, 4);
  osal_task_create("control", &EcuFirmware::control_task, this, 2048, 3);
  osal_task_create("actuator", &EcuFirmware::actuator_task, this, 1024, 3);
  osal_task_create("diag", &EcuFirmware::diag_task, this, 4096, 2);

  // CAN controller out of reset; RX interrupt feeds the RX queue
  can_.rx_isr = [this](const canfd::Frame& f, uint64_t) {
    if (!osal_queue_send_from_isr(rx_q_, &f)) ++stats_.rx_overflow;
  };
  can_.enabled = true;

  if (cfg_.wdg != WdgMode::Off) osal_hw_watchdog_start(cfg_.wdg_timeout_ms);
}

void EcuFirmware::check_faults(int t) {
  if (hang_[t]) {
    for (;;) osal_cpu_work_us(1000);  // runaway loop, never yields voluntarily
  }
  if (deadlock_[t]) {
    uint8_t d;
    osal_queue_receive(dead_q_, &d, OSAL_WAIT_FOREVER);  // waits for a resource that never comes
  }
}

bool EcuFirmware::enqueue_tx(const canfd::Frame& f, uint32_t deadline_ms) {
  TxItem it{f, deadline_ms};
  if (!osal_queue_send(tx_q_, &it, OSAL_NO_WAIT)) {
    ++stats_.tx_queue_full;
    return false;
  }
  return true;
}

void EcuFirmware::wdg_task(void* p) {
  auto* fw = static_cast<EcuFirmware*>(p);
  uint32_t wake = osal_tick_ms();
  for (;;) {
    osal_delay_until(&wake, fw->cfg_.wdg_period_ms);
    fw->check_faults(kWdg);
    osal_cpu_work_us(10);
    if (fw->cfg_.wdg == WdgMode::Off) continue;
    if (fw->cfg_.wdg == WdgMode::Unconditional) {
      osal_hw_watchdog_kick();
      ++fw->stats_.wdg_kicks;
      continue;
    }
    const uint32_t now = osal_tick_ms();
    int culprit = -1;
    for (int i = 1; i < kNumTasks; ++i) {
      if (now - fw->hb_[i] > fw->hb_limit_[i]) {
        culprit = i;
        break;
      }
    }
    if (culprit < 0) {
      osal_hw_watchdog_kick();
      ++fw->stats_.wdg_kicks;
    } else {
      ++fw->stats_.supervision_failures;
      fw->nvm_.last_culprit = culprit;
      if (fw->cfg_.wdg == WdgMode::SupervisedFast) osal_system_reset();
      // Supervised: stop refreshing, the IWDG resets the MCU
    }
  }
}

void EcuFirmware::dispatch(const canfd::Frame& f) {
  const uint64_t now_us = osal_time_us();
  for (size_t i = 0; i < cfg_.rx_sensor_ids.size(); ++i) {
    if (f.id != cfg_.rx_sensor_ids[i]) continue;
    SensorMsg m;
    if (!unpack(f, m)) {
      ++stats_.rx_e2e_fail;
      return;
    }
    if (metrics_) metrics_->received(cfg_.rx_sensor_streams[i], cfg_.node, m.seq, now_us);
    if (i == 0) std::memcpy(remote_, m.value, sizeof remote_);
    return;
  }
  if (f.id == cfg_.rx_cmd_id) {
    CommandMsg c;
    if (!unpack(f, c)) {
      ++stats_.rx_e2e_fail;
      return;
    }
    if (metrics_) metrics_->received(cfg_.rx_cmd_stream, cfg_.node, c.seq, now_us);
    osal_queue_send(act_q_, &c.setpoint, OSAL_NO_WAIT);
    return;
  }
  if (f.id == cfg_.diag_req_id) osal_queue_send(diag_q_, &f, OSAL_NO_WAIT);
}

void EcuFirmware::rx_task(void* p) {
  auto* fw = static_cast<EcuFirmware*>(p);
  for (;;) {
    canfd::Frame f;
    if (osal_queue_receive(fw->rx_q_, &f, 50)) {
      osal_cpu_work_us(30);
      fw->dispatch(f);
    }
    fw->heartbeat(kRx);
    fw->check_faults(kRx);
  }
}

void EcuFirmware::tx_task(void* p) {
  auto* fw = static_cast<EcuFirmware*>(p);
  for (;;) {
    TxItem it;
    if (osal_queue_receive(fw->tx_q_, &it, 50)) {
      osal_cpu_work_us(20);
      // wait for a free TX buffer; give up once the frame is stale
      while (!fw->can_.submit(it.frame, osal_time_us())) {
        if (fw->cfg_.tx_cancel && fw->can_.cancel_lower_priority(it.frame.id)) continue;
        if (static_cast<int32_t>(osal_tick_ms() - it.deadline_ms) >= 0) {
          ++fw->stats_.stale_dropped;
          break;
        }
        osal_delay_ms(1);
        fw->heartbeat(kTx);
      }
    }
    fw->heartbeat(kTx);
    fw->check_faults(kTx);
  }
}

void EcuFirmware::sensor_task(void* p) {
  auto* fw = static_cast<EcuFirmware*>(p);
  uint32_t wake = osal_tick_ms();
  for (;;) {
    osal_delay_until(&wake, fw->cfg_.sensor_period_ms);
    fw->heartbeat(kSensor);
    fw->check_faults(kSensor);
    osal_cpu_work_us(150);  // ADC conversion + filtering
    const uint32_t now = osal_tick_ms();
    fw->sensors_.sample(now, fw->local_);
    bool range = false, rate = false, stuck = false;
    for (int i = 0; i < 3; ++i) {
      const auto r = fw->plaus_[i].check(fw->local_[i]);
      range |= r.range_fail;
      rate |= r.rate_fail;
      stuck |= r.stuck_fail;
    }
    const auto snap = fw->snapshot();
    fw->nvm_.dtc.report(DtcId::SensorRange, range, snap);
    fw->nvm_.dtc.report(DtcId::SensorRate, rate, snap);
    fw->nvm_.dtc.report(DtcId::SensorStuck, stuck, snap);
    fw->nvm_.dtc.report(DtcId::CanBusOff, fw->can_.state() == canfd::ErrState::BusOff, snap);

    SensorMsg m{};
    m.seq = fw->nvm_.sensor_seq++;
    m.t_us = osal_time_us();
    std::memcpy(m.value, fw->local_, sizeof m.value);
    m.flags = static_cast<uint8_t>((range ? 1 : 0) | (rate ? 2 : 0) | (stuck ? 4 : 0));
    if (fw->metrics_) fw->metrics_->produced(fw->cfg_.sensor_stream, m.seq, m.t_us);
    fw->enqueue_tx(pack(fw->cfg_.sensor_id, m), now + fw->cfg_.sensor_period_ms);
  }
}

void EcuFirmware::control_task(void* p) {
  auto* fw = static_cast<EcuFirmware*>(p);
  uint32_t wake = osal_tick_ms();
  for (;;) {
    osal_delay_until(&wake, fw->cfg_.cmd_period_ms);
    fw->heartbeat(kControl);
    fw->check_faults(kControl);
    osal_cpu_work_us(80);
    CommandMsg c{};
    c.seq = fw->nvm_.cmd_seq++;
    c.t_us = osal_time_us();
    // simple feed-forward + proportional law on local and remote sensors
    c.setpoint = 20.f * fw->local_[0] + 5.f * (fw->remote_[1] - fw->local_[1]);
    if (fw->metrics_) fw->metrics_->produced(fw->cfg_.cmd_stream, c.seq, c.t_us);
    fw->enqueue_tx(pack(fw->cfg_.cmd_id, c), osal_tick_ms() + fw->cfg_.cmd_period_ms);
  }
}

void EcuFirmware::actuator_task(void* p) {
  auto* fw = static_cast<EcuFirmware*>(p);
  for (;;) {
    float sp;
    if (osal_queue_receive(fw->act_q_, &sp, 10)) fw->act_.command(sp, osal_tick_ms());
    osal_cpu_work_us(40);
    fw->act_.update(osal_tick_ms());
    fw->nvm_.dtc.report(DtcId::CommandTimeout, fw->act_.timed_out(), fw->snapshot());
    fw->heartbeat(kActuator);
    fw->check_faults(kActuator);
  }
}

bool EcuFirmware::read_did(uint16_t did, std::vector<uint8_t>& out) {
  auto put32 = [&out](uint32_t v) {
    for (int s = 24; s >= 0; s -= 8) out.push_back(static_cast<uint8_t>(v >> s));
  };
  switch (did) {
    case uds::kDidNodeId: out.push_back(static_cast<uint8_t>(cfg_.node)); return true;
    case uds::kDidResetCount: put32(nvm_.boots); return true;
    case uds::kDidUptime: put32(osal_tick_ms() - boot_ms_); return true;
    case uds::kDidResetCause: out.push_back(static_cast<uint8_t>(osal_reset_cause())); return true;
    case uds::kDidSensorValues:
      for (float v : local_) put32(static_cast<uint32_t>(static_cast<int32_t>(v * 1000.f)));
      return true;
    default: return false;
  }
}

void EcuFirmware::diag_task(void* p) {
  auto* fw = static_cast<EcuFirmware*>(p);
  for (;;) {
    canfd::Frame f;
    if (osal_queue_receive(fw->diag_q_, &f, 100)) {
      osal_cpu_work_us(200);
      uint8_t buf[64];
      const size_t n = unpack_diag(f, buf, sizeof buf);
      if (n > 0) {
        const auto rsp = fw->uds_.handle(std::vector<uint8_t>(buf, buf + n));
        if (!rsp.empty()) fw->enqueue_tx(pack_diag(fw->cfg_.diag_resp_id, rsp.data(), rsp.size()), osal_tick_ms() + 50);
        if (fw->uds_.reset_requested()) {
          osal_delay_ms(10);  // let the positive response go out
          osal_system_reset();
        }
      }
    }
    fw->heartbeat(kDiag);
    fw->check_faults(kDiag);
  }
}

}  // namespace ecu
