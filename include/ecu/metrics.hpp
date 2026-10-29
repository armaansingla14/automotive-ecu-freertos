#pragma once
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace ecu {

/// Measurement probe (host simulation only): records every application
/// message instance produced and every reception at a consumer's
/// application layer, so delivery and end-to-end latency can be computed.
class Metrics {
 public:
  struct Stream {
    std::string name;
    int producer;
    uint32_t period_ms;
    uint32_t can_id;
    std::vector<int> consumers;
    std::vector<std::pair<uint32_t, uint64_t>> produced;  ///< (seq, t_us)
    std::unordered_map<uint64_t, uint64_t> received;      ///< (consumer<<32 | seq) -> t_us
  };

  int add_stream(const std::string& name, int producer, uint32_t period_ms, uint32_t can_id,
                 std::vector<int> consumers);
  void produced(int stream, uint32_t seq, uint64_t t_us);
  void received(int stream, int consumer, uint32_t seq, uint64_t t_us);
  std::vector<Stream>& streams() { return streams_; }
  const std::vector<Stream>& streams() const { return streams_; }

  struct Summary {
    uint64_t expected = 0;     ///< (instance, consumer) pairs
    uint64_t delivered = 0;    ///< received at all
    uint64_t in_deadline = 0;  ///< received within one period
    std::vector<double> latency_ms;
    double delivery_rate() const { return expected ? double(in_deadline) / double(expected) : 0.0; }
    double any_rate() const { return expected ? double(delivered) / double(expected) : 0.0; }
    double pct(double p) const;  ///< latency percentile (sorted on demand)
    double max_latency() const;
  };
  /// Instances produced in [t0, t1) (us); filter on period (0 = all).
  Summary summarize(uint64_t t0, uint64_t t1, uint32_t period_filter = 0, int producer_filter = -1) const;

 private:
  std::vector<Stream> streams_;
};

}  // namespace ecu
