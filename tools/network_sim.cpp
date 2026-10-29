// Experiments on the simulated 20-ECU CAN FD network.
//
//   network_sim [all|load|ber|ids|txpolicy|watchdog|confinement] [--sim-ms N] [--out DIR]
//
// Every number written to results/ comes from running this program.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include "ecu/network.hpp"
#include "sim/kernel.hpp"

using namespace ecu;

namespace {

uint64_t g_sim_ms = 30000;
std::string g_out = "results";
const uint64_t kWarmupMs = 1000;

struct RunResult {
  double util = 0;
  Metrics::Summary all, p10, p100;
  uint64_t error_frames = 0, undetected = 0, frames_ok = 0, bus_off = 0;
  uint64_t stale = 0, txq_full = 0, rx_overflow = 0, e2e_fail = 0;
  uint64_t bg_sent = 0, bg_dropped = 0;
  TesterStats tester;
  uint64_t ctx_switches = 0;
};

RunResult run(const NetConfig& cfg, uint64_t sim_ms) {
  Network net(cfg);
  net.run_until_ms(sim_ms);
  RunResult r;
  const uint64_t t0 = kWarmupMs * 1000, t1 = (sim_ms - 200) * 1000;  // leave time for in-flight frames
  r.util = net.bus().utilization(sim_ms * 1000);
  r.all = net.metrics().summarize(t0, t1);
  r.p10 = net.metrics().summarize(t0, t1, 10);
  r.p100 = net.metrics().summarize(t0, t1, 100);
  r.error_frames = net.bus().error_frames;
  r.undetected = net.bus().undetected_errors;
  r.frames_ok = net.bus().frames_ok;
  for (int i = 0; i < net.size(); ++i) {
    r.bus_off += net.bus().controller(static_cast<size_t>(i)).bus_off_events;
    r.stale += net.ecu(i).stats().stale_dropped;
    r.txq_full += net.ecu(i).stats().tx_queue_full;
    r.rx_overflow += net.ecu(i).stats().rx_overflow;
    r.e2e_fail += net.ecu(i).stats().rx_e2e_fail;
  }
  r.bg_sent = net.background_sent();
  r.bg_dropped = net.background_dropped();
  r.tester = net.tester();
  r.ctx_switches = sim::Kernel::get().context_switches();
  return r;
}

double pct(double x) { return 100.0 * x; }

void exp_load(FILE* md) {
  std::ofstream csv(g_out + "/load_sweep.csv");
  csv << "ber,bg_load,bus_util,expected,delivered_any,in_deadline,delivery_pct,p50_ms,p99_ms,max_ms,"
         "delivery_10ms_pct,p99_10ms,delivery_100ms_pct,p99_100ms,error_frames,stale_drops,txq_full,bg_dropped\n";
  std::fprintf(md, "## 1. Bus load sweep\n\n");
  std::fprintf(md, "20 ECUs, 500 kbit/s arbitration / 2 Mbit/s data phase, %llu s simulated per point, first 1 s "
                   "discarded. Background traffic: 8 extra 64-byte streams (IDs 0x140-0x147).\n\n",
               static_cast<unsigned long long>(g_sim_ms / 1000));
  for (double ber : {0.0, 1e-5}) {
    std::fprintf(md, "### BER = %g\n\n", ber);
    std::fprintf(md, "| Background | Bus util. | Delivered in deadline | p50 / p99 / max latency (ms) | 10 ms msgs | "
                     "100 ms msgs | Error frames | Stale drops |\n|---:|---:|---:|---|---:|---:|---:|---:|\n");
    for (double bg : {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7}) {
      NetConfig c;
      c.ber = ber;
      c.background_load = bg;
      c.seed = 11;
      const RunResult r = run(c, g_sim_ms);
      csv << ber << ',' << bg << ',' << r.util << ',' << r.all.expected << ',' << r.all.delivered << ','
          << r.all.in_deadline << ',' << pct(r.all.delivery_rate()) << ',' << r.all.pct(0.5) << ','
          << r.all.pct(0.99) << ',' << r.all.max_latency() << ',' << pct(r.p10.delivery_rate()) << ','
          << r.p10.pct(0.99) << ',' << pct(r.p100.delivery_rate()) << ',' << r.p100.pct(0.99) << ','
          << r.error_frames << ',' << r.stale << ',' << r.txq_full << ',' << r.bg_dropped << '\n';
      std::fprintf(md, "| %.0f %% | %.1f %% | %.3f %% | %.2f / %.2f / %.2f | %.3f %% | %.3f %% | %llu | %llu |\n",
                   100 * bg, 100 * r.util, pct(r.all.delivery_rate()), r.all.pct(0.5), r.all.pct(0.99),
                   r.all.max_latency(), pct(r.p10.delivery_rate()), pct(r.p100.delivery_rate()),
                   static_cast<unsigned long long>(r.error_frames), static_cast<unsigned long long>(r.stale));
      std::printf("load ber=%g bg=%.1f util=%.3f delivery=%.4f p99=%.2f\n", ber, bg, r.util, r.all.delivery_rate(),
                  r.all.pct(0.99));
      std::fflush(stdout);
    }
    std::fprintf(md, "\n");
  }
}

void exp_ber(FILE* md) {
  std::ofstream csv(g_out + "/ber_sweep.csv");
  csv << "ber,bus_util,delivery_pct,delivered_any_pct,p99_ms,max_ms,frames_ok,error_frames,error_frame_ratio,"
         "undetected,bus_off,e2e_fail,stale_drops\n";
  std::fprintf(md, "## 2. Bit-error-rate sweep (background load 20 %%)\n\n");
  std::fprintf(md, "| BER | Bus util. | Delivered in deadline | Delivered at all | p99 / max latency (ms) | Error frames "
                   "| Undetected corruptions | Bus-off events |\n|---:|---:|---:|---:|---|---:|---:|---:|\n");
  for (double ber : {0.0, 1e-7, 1e-6, 1e-5, 1e-4, 3e-4, 1e-3}) {
    NetConfig c;
    c.ber = ber;
    c.background_load = 0.2;
    c.seed = 23;
    const RunResult r = run(c, g_sim_ms);
    const double efr = r.frames_ok ? double(r.error_frames) / double(r.frames_ok + r.error_frames) : 0.0;
    csv << ber << ',' << r.util << ',' << pct(r.all.delivery_rate()) << ',' << pct(r.all.any_rate()) << ','
        << r.all.pct(0.99) << ',' << r.all.max_latency() << ',' << r.frames_ok << ',' << r.error_frames << ','
        << efr << ',' << r.undetected << ',' << r.bus_off << ',' << r.e2e_fail << ',' << r.stale << '\n';
    std::fprintf(md, "| %g | %.1f %% | %.3f %% | %.3f %% | %.2f / %.2f | %llu (%.2f %%) | %llu | %llu |\n", ber,
                 100 * r.util, pct(r.all.delivery_rate()), pct(r.all.any_rate()), r.all.pct(0.99), r.all.max_latency(),
                 static_cast<unsigned long long>(r.error_frames), 100 * efr, static_cast<unsigned long long>(r.undetected),
                 static_cast<unsigned long long>(r.bus_off));
    std::printf("ber=%g delivery=%.4f errors=%llu\n", ber, r.all.delivery_rate(),
                static_cast<unsigned long long>(r.error_frames));
    std::fflush(stdout);
  }
  std::fprintf(md, "\n");
}

void exp_ids(FILE* md) {
  std::ofstream csv(g_out + "/id_assignment.csv");
  csv << "assignment,bg_load,bus_util,delivery_pct,delivery_10ms_pct,p99_10ms,max_10ms,delivery_100ms_pct\n";
  std::fprintf(md, "## 3. Identifier assignment: deadline-monotonic vs arbitrary\n\n");
  std::fprintf(md, "Arbitrary = sensor IDs drawn at random from 0x100-0x18F, interleaved with the background IDs. "
                   "Mean of 3 seeds.\n\n");
  std::fprintf(md, "| Background | Assignment | Bus util. | Delivered in deadline (all) | 10 ms msgs in deadline | "
                   "10 ms p99 / max (ms) |\n|---:|---|---:|---:|---:|---|\n");
  for (double bg : {0.3, 0.5, 0.6, 0.7}) {
    for (bool dm : {true, false}) {
      double util = 0, d = 0, d10 = 0, p99 = 0, mx = 0, d100 = 0;
      const int seeds = 3;
      for (int s = 0; s < seeds; ++s) {
        NetConfig c;
        c.background_load = bg;
        c.deadline_monotonic_ids = dm;
        c.seed = 100 + static_cast<uint64_t>(s);
        const RunResult r = run(c, g_sim_ms);
        util += r.util / seeds;
        d += r.all.delivery_rate() / seeds;
        d10 += r.p10.delivery_rate() / seeds;
        p99 += r.p10.pct(0.99) / seeds;
        mx = std::max(mx, r.p10.max_latency());
        d100 += r.p100.delivery_rate() / seeds;
      }
      csv << (dm ? "deadline_monotonic" : "arbitrary") << ',' << bg << ',' << util << ',' << pct(d) << ','
          << pct(d10) << ',' << p99 << ',' << mx << ',' << pct(d100) << '\n';
      std::fprintf(md, "| %.0f %% | %s | %.1f %% | %.3f %% | %.3f %% | %.2f / %.2f |\n", 100 * bg,
                   dm ? "deadline-monotonic" : "arbitrary", 100 * util, pct(d), pct(d10), p99, mx);
      std::printf("ids bg=%.1f dm=%d d=%.4f d10=%.4f\n", bg, dm, d, d10);
      std::fflush(stdout);
    }
  }
  std::fprintf(md, "\n");
}

void exp_txpolicy(FILE* md) {
  std::ofstream csv(g_out + "/tx_policy.csv");
  csv << "tx_policy,bg_load,bus_util,delivery_pct,stale_drops,diag_positive,diag_requests\n";
  std::fprintf(md, "## 3b. TX buffer policy near saturation\n\n");
  std::fprintf(md, "`wait` = the TX task waits for a free controller buffer (FIFO); `cancel` = it cancels a pending "
                   "lower-priority frame (typically a diagnostic response) to make room. Seed 11.\n\n");
  std::fprintf(md, "| Background | TX policy | Bus util. | Delivered in deadline | Stale drops | Diag responses / requests "
                   "|\n|---:|---|---:|---:|---:|---:|\n");
  for (double bg : {0.5, 0.6, 0.7, 0.8}) {
    for (bool cancel : {false, true}) {
      NetConfig c;
      c.background_load = bg;
      c.tx_cancel = cancel;
      c.seed = 11;
      const RunResult r = run(c, g_sim_ms);
      csv << (cancel ? "cancel" : "wait") << ',' << bg << ',' << r.util << ',' << pct(r.all.delivery_rate()) << ','
          << r.stale << ',' << r.tester.positive << ',' << r.tester.requests << '\n';
      std::fprintf(md, "| %.0f %% | %s | %.1f %% | %.3f %% | %llu | %llu / %llu |\n", 100 * bg, cancel ? "cancel" : "wait",
                   100 * r.util, pct(r.all.delivery_rate()), static_cast<unsigned long long>(r.stale),
                   static_cast<unsigned long long>(r.tester.positive), static_cast<unsigned long long>(r.tester.requests));
      std::printf("txpolicy bg=%.1f cancel=%d d=%.4f\n", bg, cancel, r.all.delivery_rate());
      std::fflush(stdout);
    }
  }
  std::fprintf(md, "\n");
}

struct FaultSpec {
  uint64_t t_ms;
  int node;
  int task;
  bool hang;
};

void exp_watchdog(FILE* md) {
  std::ofstream csv(g_out + "/watchdog.csv");
  csv << "mode,seed,fault_idx,t_fault_ms,node,task,kind,reset,detect_ms,recover_ms\n";
  std::ofstream sum(g_out + "/watchdog_summary.csv");
  sum << "mode,faults,recovered,detect_mean_ms,detect_max_ms,recover_mean_ms,recover_p95_ms,recover_max_ms,"
         "delivery_pct,resets,false_resets\n";
  static const char* task_names[] = {"wdg", "can_rx", "can_tx", "sensor", "control", "actuator", "diag"};
  const uint64_t sim_ms = std::max<uint64_t>(g_sim_ms * 2, 60000);
  std::fprintf(md, "## 4. Watchdog recovery\n\n");
  std::fprintf(md, "%llu s simulated per seed, 3 seeds, 12 transient software faults per run (random node, random task "
                   "among can_rx/can_tx/sensor/control/actuator/diag, half runaway loops, half deadlocks), background "
                   "load 10 %%, BER 0. Detection = fault to MCU reset; recovery = fault to the first sensor frame of "
                   "that ECU delivered after the reset.\n\n",
               static_cast<unsigned long long>(sim_ms / 1000));
  std::fprintf(md, "| Watchdog mode | Faults | Recovered | Detection mean / max (ms) | Recovery mean / p95 / max (ms) | "
                   "Network delivery in deadline | Resets without fault |\n|---|---:|---:|---|---|---:|---:|\n");
  for (WdgMode mode : {WdgMode::Off, WdgMode::Unconditional, WdgMode::Supervised, WdgMode::SupervisedFast}) {
    std::vector<double> det, rec;
    int faults = 0, recovered = 0;
    uint64_t exp = 0, inl = 0, resets = 0, false_resets = 0;
    for (int seed = 0; seed < 3; ++seed) {
      NetConfig c;
      c.wdg = mode;
      c.background_load = 0.1;
      c.seed = 500 + static_cast<uint64_t>(seed);
      Network net(c);
      std::mt19937_64 rng(900 + static_cast<uint64_t>(seed));
      std::vector<FaultSpec> fl;
      for (int i = 0; i < 12; ++i) {
        FaultSpec f;
        f.t_ms = 5000 + static_cast<uint64_t>(i) * ((sim_ms - 10000) / 12) + rng() % 1000;
        f.node = static_cast<int>(rng() % 20);
        f.task = 1 + static_cast<int>(rng() % 6);
        f.hang = (i % 2) == 0;
        fl.push_back(f);
        if (f.hang) {
          net.schedule_hang(f.t_ms, f.node, f.task);
        } else {
          net.schedule_deadlock(f.t_ms, f.node, f.task);
        }
      }
      net.run_until_ms(sim_ms);
      const auto s = net.metrics().summarize(kWarmupMs * 1000, (sim_ms - 200) * 1000);
      exp += s.expected;
      inl += s.in_deadline;
      // per fault: reset after the fault and first delivery of the node's sensor stream after boot
      for (size_t i = 0; i < fl.size(); ++i) {
        const FaultSpec& f = fl[i];
        ++faults;
        const sim::Node* node = sim::Kernel::get().node(f.node);
        int64_t t_reset = -1;
        for (uint64_t tr : node->reset_times_us) {
          if (tr >= f.t_ms * 1000) {
            t_reset = static_cast<int64_t>(tr);
            break;
          }
        }
        int64_t t_rec = -1;
        if (t_reset >= 0) {
          const auto& st = net.metrics().streams()[static_cast<size_t>(2 * f.node)];  // sensor stream of the node
          for (const auto& kv : st.received) {
            if (static_cast<int64_t>(kv.second) > t_reset && (t_rec < 0 || static_cast<int64_t>(kv.second) < t_rec))
              t_rec = static_cast<int64_t>(kv.second);
          }
        }
        const double d_ms = t_reset >= 0 ? (t_reset / 1000.0 - static_cast<double>(f.t_ms)) : -1;
        const double r_ms = t_rec >= 0 ? (t_rec / 1000.0 - static_cast<double>(f.t_ms)) : -1;
        if (t_rec >= 0) {
          ++recovered;
          det.push_back(d_ms);
          rec.push_back(r_ms);
        }
        csv << to_string(mode) << ',' << seed << ',' << i << ',' << f.t_ms << ',' << f.node << ','
            << task_names[f.task] << ',' << (f.hang ? "runaway" : "deadlock") << ',' << (t_reset >= 0) << ','
            << d_ms << ',' << r_ms << '\n';
      }
      for (int i = 0; i < net.size(); ++i) {
        const sim::Node* node = sim::Kernel::get().node(i);
        resets += node->resets;
        // resets of nodes that never had a fault injected
        bool had_fault = false;
        for (const auto& f : fl) had_fault |= f.node == i;
        if (!had_fault) false_resets += node->resets;
      }
    }
    auto mean = [](const std::vector<double>& v) {
      double s = 0;
      for (double x : v) s += x;
      return v.empty() ? 0.0 : s / static_cast<double>(v.size());
    };
    auto pctl = [](std::vector<double> v, double p) {
      if (v.empty()) return 0.0;
      std::sort(v.begin(), v.end());
      return v[static_cast<size_t>(p * static_cast<double>(v.size() - 1) + 0.5)];
    };
    const double delivery = exp ? double(inl) / double(exp) : 0.0;
    sum << to_string(mode) << ',' << faults << ',' << recovered << ',' << mean(det) << ',' << pctl(det, 1.0) << ','
        << mean(rec) << ',' << pctl(rec, 0.95) << ',' << pctl(rec, 1.0) << ',' << pct(delivery) << ',' << resets << ','
        << false_resets << '\n';
    if (recovered > 0) {
      std::fprintf(md, "| %s | %d | %d | %.0f / %.0f | %.0f / %.0f / %.0f | %.3f %% | %llu |\n", to_string(mode), faults,
                   recovered, mean(det), pctl(det, 1.0), mean(rec), pctl(rec, 0.95), pctl(rec, 1.0), pct(delivery),
                   static_cast<unsigned long long>(false_resets));
    } else {
      std::fprintf(md, "| %s | %d | 0 | - | - | %.3f %% | %llu |\n", to_string(mode), faults, pct(delivery),
                   static_cast<unsigned long long>(false_resets));
    }
    std::printf("wdg %s faults=%d recovered=%d delivery=%.4f\n", to_string(mode), faults, recovered, delivery);
    std::fflush(stdout);
  }
  std::fprintf(md, "\n");
}

void exp_confinement(FILE* md) {
  std::ofstream csv(g_out + "/confinement.csv");
  csv << "scenario,faulty_node_bus_off,error_frames,delivery_others_pct,delivery_faulty_pct,p99_others_ms\n";
  std::fprintf(md, "## 5. Fault confinement: one ECU with a defective transceiver\n\n");
  std::fprintf(md, "ECU 3 corrupts its own transmissions (5 %% BER on its frames) from t = 5 s to 15 s; background 20 %%, "
                   "%llu s run.\n\n| Scenario | Bus-off events (ECU 3) | Error frames | Delivery, other ECUs | Delivery, ECU 3 "
                   "| p99 latency, others (ms) |\n|---|---:|---:|---:|---:|---:|\n",
               static_cast<unsigned long long>(g_sim_ms / 1000));
  for (bool faulty : {false, true}) {
    NetConfig c;
    c.background_load = 0.2;
    c.seed = 77;
    Network net(c);
    if (faulty) net.schedule_tx_fault(5000, 15000, 3);
    net.run_until_ms(g_sim_ms);
    // others = all streams not produced by node 3, during the fault window
    Metrics::Summary others;
    for (int p = 0; p < 20; ++p) {
      if (p == 3) continue;
      auto s = net.metrics().summarize(5000000, 15000000, 0, p);
      others.expected += s.expected;
      others.in_deadline += s.in_deadline;
      others.delivered += s.delivered;
      others.latency_ms.insert(others.latency_ms.end(), s.latency_ms.begin(), s.latency_ms.end());
    }
    const auto s3 = net.metrics().summarize(5000000, 15000000, 0, 3);
    const uint64_t bo = net.bus().controller(3).bus_off_events;
    csv << (faulty ? "faulty_transceiver" : "healthy") << ',' << bo << ',' << net.bus().error_frames << ','
        << pct(others.delivery_rate()) << ',' << pct(s3.delivery_rate()) << ',' << others.pct(0.99) << '\n';
    std::fprintf(md, "| %s | %llu | %llu | %.3f %% | %.3f %% | %.2f |\n", faulty ? "faulty transceiver" : "healthy",
                 static_cast<unsigned long long>(bo), static_cast<unsigned long long>(net.bus().error_frames),
                 pct(others.delivery_rate()), pct(s3.delivery_rate()), others.pct(0.99));
  }
  std::fprintf(md, "\n");
}

}  // namespace

int main(int argc, char** argv) {
  std::string what = "all";
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--sim-ms") && i + 1 < argc) {
      g_sim_ms = std::strtoull(argv[++i], nullptr, 10);
    } else if (!std::strcmp(argv[i], "--out") && i + 1 < argc) {
      g_out = argv[++i];
    } else {
      what = argv[i];
    }
  }
  const std::string md_path = g_out + "/summary_" + what + ".md";
  FILE* md = std::fopen(md_path.c_str(), "w");
  if (!md) {
    std::perror(md_path.c_str());
    return 1;
  }
  std::fprintf(md, "# Simulated 20-ECU CAN FD network results\n\nGenerated by `network_sim %s --sim-ms %llu`.\n\n",
               what.c_str(), static_cast<unsigned long long>(g_sim_ms));
  if (what == "all" || what == "load") exp_load(md);
  if (what == "all" || what == "ber") exp_ber(md);
  if (what == "all" || what == "ids") exp_ids(md);
  if (what == "all" || what == "txpolicy") exp_txpolicy(md);
  if (what == "all" || what == "watchdog") exp_watchdog(md);
  if (what == "all" || what == "confinement") exp_confinement(md);
  std::fclose(md);
  std::printf("wrote %s\n", md_path.c_str());
  return 0;
}
