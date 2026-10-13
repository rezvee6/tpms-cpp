#include "harness.hpp"

#include <algorithm>
#include <chrono>
#include <new>

namespace itpms::sim {
namespace {
constexpr std::uint32_t kChunkTicks = 500;  // 0.5 s of simulated time per timed replay
constexpr double kTaskPeriod = 1e-3;
}  // namespace

Harness::Harness(std::uint64_t seed, const DetectorConfig& cfg) : sim_(seed), det_(cfg), cfg_(cfg) {
  edge_buf_.reserve(8192);
  tick_buf_.reserve(kChunkTicks);
}

void Harness::reset(std::uint64_t seed) {
  sim_.reset(seed);
  det_.~Detector();
  new (&det_) Detector(cfg_);
  tick_count_ = 0;
  carry_ms_ = 0.0;
  perf_ = {};
}

void Harness::run_for(double sim_seconds) {
  carry_ms_ += sim_seconds * 1000.0;
  auto ticks = static_cast<std::uint64_t>(carry_ms_);
  carry_ms_ -= static_cast<double>(ticks);
  while (ticks > 0) {
    const auto n = static_cast<std::uint32_t>(std::min<std::uint64_t>(ticks, kChunkTicks));
    run_chunk(n);
    ticks -= n;
  }
}

void Harness::run_chunk(std::uint32_t ticks) {
  edge_buf_.clear();
  tick_buf_.clear();
  for (std::uint32_t i = 0; i < ticks; ++i) {
    sim_.advance(kTaskPeriod, [this](const ToothEdge& e) { edge_buf_.push_back(e); });
    Tick t{static_cast<std::uint32_t>(edge_buf_.size()), false, {}};
    if (tick_count_++ % 10 == 0) {
      t.has_signals = true;
      t.signals = VehicleSignals{sim_.sample_yaw_rate(), sim_.brake()};
    }
    tick_buf_.push_back(t);
  }

  const auto start = std::chrono::steady_clock::now();
  std::size_t e = 0;
  for (const Tick& t : tick_buf_) {
    for (; e < t.edge_end; ++e) det_.on_tooth_edge(static_cast<Wheel>(edge_buf_[e].wheel), edge_buf_[e].ticks);
    if (t.has_signals) det_.on_vehicle_signals(t.signals);
    det_.process();
  }
  const auto stop = std::chrono::steady_clock::now();

  perf_.detector_ns += static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start).count());
  perf_.sim_seconds += ticks * kTaskPeriod;
  perf_.edges += edge_buf_.size();
  perf_.task_calls += ticks;
}

}  // namespace itpms::sim
