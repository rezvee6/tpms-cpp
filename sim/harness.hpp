// Couples the simulator to the detector the way an ECU would: edges are
// captured by the ISR into the queue, a 1 ms task calls process(), CAN
// signals arrive at 100 Hz.
//
// Each chunk is simulated first and then replayed into the detector under a
// single stopwatch, so the measured detector time excludes the simulator and
// doesn't depend on a fine-grained clock (browsers coarsen performance.now).
#pragma once

#include <cstdint>
#include <vector>

#include "itpms/detector.hpp"
#include "vehicle_sim.hpp"

namespace itpms::sim {

struct PerfStats {
  double detector_ns = 0.0;  // wall time spent inside the detector
  double sim_seconds = 0.0;  // simulated time fed to it
  std::uint64_t edges = 0;
  std::uint64_t task_calls = 0;
};

class Harness {
 public:
  explicit Harness(std::uint64_t seed = 1, const DetectorConfig& cfg = {});

  void run_for(double sim_seconds);
  void reset(std::uint64_t seed);

  VehicleSim& sim() { return sim_; }
  Detector& detector() { return det_; }
  const PerfStats& perf() const { return perf_; }

 private:
  struct Tick {
    std::uint32_t edge_end;
    bool has_signals;
    VehicleSignals signals;
  };

  void run_chunk(std::uint32_t ticks);

  VehicleSim sim_;
  Detector det_;
  DetectorConfig cfg_;
  std::uint64_t tick_count_ = 0;
  double carry_ms_ = 0.0;
  PerfStats perf_{};
  std::vector<ToothEdge> edge_buf_;
  std::vector<Tick> tick_buf_;
};

}  // namespace itpms::sim
