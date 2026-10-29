#include "ecu/network.hpp"

#include <algorithm>
#include <numeric>

#include "ecu/messages.hpp"
#include "sim/kernel.hpp"

namespace ecu {

namespace {
constexpr int kBgStreams = 8;
constexpr uint32_t kBgBaseId = 0x140;
constexpr uint32_t kTesterReqBase = 0x700;
constexpr uint32_t kRespBase = 0x740;
}  // namespace

uint32_t Network::sensor_period_for(int node) {
  static const uint32_t periods[4] = {10, 20, 50, 100};
  return periods[node % 4];
}

Network::Network(const NetConfig& cfg) : cfg_(cfg), rng_(cfg.seed) {
  sim::Kernel& k = sim::Kernel::get();
  k.clear();
  canfd::BusConfig bc;
  bc.timing = cfg.timing;
  bc.ber = cfg.ber;
  bc.seed = cfg.seed * 7919 + 1;
  bus_ = std::make_unique<canfd::Bus>(k, bc);

  const int n = cfg.nodes;
  // --- identifier assignment ----------------------------------------------
  // deadline-monotonic: shorter period -> lower ID (higher priority)
  std::vector<int> order(static_cast<size_t>(n));
  std::iota(order.begin(), order.end(), 0);
  std::vector<uint32_t> sensor_id(static_cast<size_t>(n));
  if (cfg.deadline_monotonic_ids) {
    std::stable_sort(order.begin(), order.end(),
                     [](int a, int b) { return sensor_period_for(a) < sensor_period_for(b); });
    for (int r = 0; r < n; ++r) sensor_id[static_cast<size_t>(order[static_cast<size_t>(r)])] = 0x100 + static_cast<uint32_t>(r);
  } else {
    // arbitrary assignment from a wider range that interleaves with the
    // background traffic IDs (0x140..0x147)
    std::vector<uint32_t> pool;
    for (uint32_t id = 0x100; id < 0x190; ++id) {
      if (id < kBgBaseId || id >= kBgBaseId + kBgStreams) pool.push_back(id);
    }
    std::shuffle(pool.begin(), pool.end(), rng_);
    for (int i = 0; i < n; ++i) sensor_id[static_cast<size_t>(i)] = pool[static_cast<size_t>(i)];
  }

  // --- streams: node i consumes sensors of i-1 and i-5, command of i-1 -------
  std::vector<int> sensor_stream(static_cast<size_t>(n)), cmd_stream(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    const int c1 = (i + 1) % n, c2 = (i + 5) % n;
    sensor_stream[static_cast<size_t>(i)] =
        metrics_.add_stream("sensor" + std::to_string(i), i, sensor_period_for(i), sensor_id[static_cast<size_t>(i)], {c1, c2});
    cmd_stream[static_cast<size_t>(i)] = metrics_.add_stream("cmd" + std::to_string(i), i, 50, 0x080u + static_cast<uint32_t>(i), {c1});
  }

  for (int i = 0; i < n; ++i) {
    EcuConfig ec;
    ec.node = i;
    ec.sensor_period_ms = sensor_period_for(i);
    ec.sensor_id = sensor_id[static_cast<size_t>(i)];
    ec.cmd_id = 0x080u + static_cast<uint32_t>(i);
    const int p1 = (i - 1 + n) % n, p5 = (i - 5 + n) % n;
    ec.rx_sensor_ids = {sensor_id[static_cast<size_t>(p1)], sensor_id[static_cast<size_t>(p5)]};
    ec.rx_sensor_streams = {sensor_stream[static_cast<size_t>(p1)], sensor_stream[static_cast<size_t>(p5)]};
    ec.rx_cmd_id = 0x080u + static_cast<uint32_t>(p1);
    ec.rx_cmd_stream = cmd_stream[static_cast<size_t>(p1)];
    ec.diag_req_id = kTesterReqBase + static_cast<uint32_t>(i);
    ec.diag_resp_id = kRespBase + static_cast<uint32_t>(i);
    ec.sensor_stream = sensor_stream[static_cast<size_t>(i)];
    ec.cmd_stream = cmd_stream[static_cast<size_t>(i)];
    ec.wdg = cfg.wdg;
    ec.tx_cancel = cfg.tx_cancel;
    ec.seed = cfg.seed * 1000 + static_cast<uint64_t>(i);
    canfd::Controller& ctrl = bus_->attach(i);
    ctrl.enabled = false;  // held in reset until the firmware boots
    auto fw = std::make_unique<EcuFirmware>(ec, ctrl, &metrics_);
    EcuFirmware* fwp = fw.get();
    sim::Node& node = k.add_node("ecu" + std::to_string(i), [fwp]() { fwp->boot(); });
    node.user = fwp;
    node.on_reset = [fwp](sim::Node&) { fwp->on_reset(); };
    // ECUs come out of reset at slightly different times (0..20 ms), so
    // their periodic tasks are not phase-aligned
    node.boot_delay_us = 20000;
    node.power_on_us = 1 + std::uniform_int_distribution<uint64_t>(0, 20000)(rng_);
    ecus_.push_back(std::move(fw));
  }

  bg_ctrl_ = &bus_->attach(1000);
  bg_ctrl_->tx_buffers = 8;
  bg_ctrl_->add_filter(0x7FF, 0x7FF);  // receives nothing of interest
  tester_ctrl_ = &bus_->attach(1001);
  tester_ctrl_->add_filter(kRespBase, 0x7C0);  // 0x740..0x77F
  tester_ctrl_->rx_isr = [this](const canfd::Frame& f, uint64_t t) {
    if (!tester_waiting_ || f.id != kRespBase + static_cast<uint32_t>(tester_target_)) return;
    tester_waiting_ = false;
    tester_.response_ms.push_back(static_cast<double>(t - tester_sent_us_) / 1000.0);
    if (f.data[1] == 0x7F) {
      ++tester_.negative;
    } else {
      ++tester_.positive;
    }
  };

  k.start();
  if (cfg.background_load > 0.0) start_background();
  if (cfg.tester) k.at(200000, [this]() { tester_poll(); });
}

Network::~Network() { sim::Kernel::get().clear(); }

uint64_t Network::now_ms() const { return sim::Kernel::get().now_us() / 1000; }

void Network::run_until_ms(uint64_t t_ms) { sim::Kernel::get().run_until(t_ms * 1000); }

void Network::start_background() {
  canfd::Frame f;
  f.len = 64;
  for (int i = 0; i < 64; ++i) f.data[static_cast<size_t>(i)] = static_cast<uint8_t>(rng_());
  const auto w = canfd::encode(f);
  const double frame_us = static_cast<double>(canfd::frame_duration_ns(w, true, cfg_.timing)) / 1000.0;
  // each stream sends one frame per period; total = load * capacity
  const double period_us = kBgStreams * frame_us / cfg_.background_load;
  for (int s = 0; s < kBgStreams; ++s) {
    const uint64_t start = 100000 + static_cast<uint64_t>(std::uniform_real_distribution<double>(0, period_us)(rng_));
    sim::Kernel::get().at(start, [this, s, period_us]() { bg_send(s, static_cast<uint64_t>(period_us)); });
  }
}

void Network::bg_send(int stream, uint64_t period_us) {
  sim::Kernel& k = sim::Kernel::get();
  canfd::Frame f;
  f.id = kBgBaseId + static_cast<uint32_t>(stream);
  f.len = 64;
  for (int i = 0; i < 64; ++i) f.data[static_cast<size_t>(i)] = static_cast<uint8_t>(rng_());
  if (bg_ctrl_->submit(f, k.now_us())) {
    ++bg_sent_;
  } else {
    ++bg_dropped_;
  }
  // +-10 % jitter keeps the sources from phase-locking
  const double j = std::uniform_real_distribution<double>(0.9, 1.1)(rng_);
  k.at(k.now_us() + static_cast<uint64_t>(static_cast<double>(period_us) * j),
       [this, stream, period_us]() { bg_send(stream, period_us); });
}

void Network::tester_poll() {
  sim::Kernel& k = sim::Kernel::get();
  if (tester_waiting_) ++tester_.no_response;
  tester_target_ = (tester_target_ + 1) % cfg_.nodes;
  static const std::vector<std::vector<uint8_t>> reqs = {
      {0x19, 0x02, 0xFF},        // read DTCs by status mask
      {0x22, 0xF1, 0x8C},        // read node id
      {0x19, 0x01, 0x08},        // number of confirmed DTCs
      {0x3E, 0x00},              // tester present
      {0x22, 0x01, 0x01},        // reset counter
  };
  const auto& r = reqs[static_cast<size_t>(tester_step_++ % static_cast<int>(reqs.size()))];
  const canfd::Frame f = pack_diag(kTesterReqBase + static_cast<uint32_t>(tester_target_), r.data(), r.size());
  ++tester_.requests;
  tester_waiting_ = tester_ctrl_->submit(f, k.now_us());
  if (!tester_waiting_) ++tester_.no_response;
  tester_sent_us_ = k.now_us();
  k.at(k.now_us() + 100000, [this]() { tester_poll(); });
}

void Network::schedule_hang(uint64_t t_ms, int node, int task) {
  sim::Kernel::get().at(t_ms * 1000, [this, node, task]() { ecus_[static_cast<size_t>(node)]->inject_hang(task); });
}

void Network::schedule_deadlock(uint64_t t_ms, int node, int task) {
  sim::Kernel::get().at(t_ms * 1000, [this, node, task]() { ecus_[static_cast<size_t>(node)]->inject_deadlock(task); });
}

void Network::schedule_tx_fault(uint64_t t0_ms, uint64_t t1_ms, int node) {
  canfd::Controller* c = &bus_->controller(static_cast<size_t>(node));
  sim::Kernel::get().at(t0_ms * 1000, [c]() { c->tx_fault = true; });
  sim::Kernel::get().at(t1_ms * 1000, [c]() { c->tx_fault = false; });
}

}  // namespace ecu
