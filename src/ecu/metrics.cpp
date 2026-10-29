#include "ecu/metrics.hpp"

#include <algorithm>

namespace ecu {

int Metrics::add_stream(const std::string& name, int producer, uint32_t period_ms, uint32_t can_id,
                        std::vector<int> consumers) {
  Stream s;
  s.name = name;
  s.producer = producer;
  s.period_ms = period_ms;
  s.can_id = can_id;
  s.consumers = std::move(consumers);
  streams_.push_back(std::move(s));
  return static_cast<int>(streams_.size() - 1);
}

void Metrics::produced(int stream, uint32_t seq, uint64_t t_us) {
  streams_[static_cast<size_t>(stream)].produced.push_back({seq, t_us});
}

void Metrics::received(int stream, int consumer, uint32_t seq, uint64_t t_us) {
  auto& m = streams_[static_cast<size_t>(stream)].received;
  const uint64_t key = (uint64_t(uint32_t(consumer)) << 32) | seq;
  m.emplace(key, t_us);  // first reception counts
}

double Metrics::Summary::pct(double p) const {
  if (latency_ms.empty()) return 0.0;
  std::vector<double> v = latency_ms;
  const size_t k = static_cast<size_t>(p * static_cast<double>(v.size() - 1) + 0.5);
  std::nth_element(v.begin(), v.begin() + static_cast<long>(k), v.end());
  return v[k];
}

double Metrics::Summary::max_latency() const {
  return latency_ms.empty() ? 0.0 : *std::max_element(latency_ms.begin(), latency_ms.end());
}

Metrics::Summary Metrics::summarize(uint64_t t0, uint64_t t1, uint32_t period_filter, int producer_filter) const {
  Summary s;
  for (const auto& st : streams_) {
    if (period_filter && st.period_ms != period_filter) continue;
    if (producer_filter >= 0 && st.producer != producer_filter) continue;
    const uint64_t deadline_us = uint64_t{st.period_ms} * 1000;
    for (const auto& [seq, t] : st.produced) {
      if (t < t0 || t >= t1) continue;
      for (int c : st.consumers) {
        ++s.expected;
        const uint64_t key = (uint64_t(uint32_t(c)) << 32) | seq;
        auto it = st.received.find(key);
        if (it == st.received.end()) continue;
        ++s.delivered;
        const uint64_t lat = it->second - t;
        if (lat <= deadline_us) ++s.in_deadline;
        s.latency_ms.push_back(static_cast<double>(lat) / 1000.0);
      }
    }
  }
  return s;
}

}  // namespace ecu
