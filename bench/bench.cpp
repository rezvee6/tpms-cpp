// Detector CPU benchmark. Edges are recorded from the simulator first, then
// replayed into the detector exactly as the ECU would see them (1 ms task),
// timing every task invocation to get mean and worst-case latency.
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <memory>
#include <vector>

#include "itpms/detector.hpp"
#include "itpms/fft.hpp"
#include "vehicle_sim.hpp"

using namespace itpms;
using namespace itpms::sim;
using Clock = std::chrono::steady_clock;

namespace {

struct Recording {
  std::vector<ToothEdge> edges;
  std::vector<std::uint32_t> tick_end;  // edges[0..tick_end[i]) arrived before task call i
  std::vector<VehicleSignals> signals;  // per task call (applied every 10th)
  double seconds;
};

Recording record(Scenario sc, double seconds, std::uint64_t seed) {
  Recording r;
  r.seconds = seconds;
  VehicleSim sim(seed);
  sim.set_scenario(sc);
  const auto ticks = static_cast<std::size_t>(seconds * 1000.0);
  r.edges.reserve(ticks * 8);
  r.tick_end.reserve(ticks);
  r.signals.reserve(ticks);
  for (std::size_t i = 0; i < ticks; ++i) {
    sim.advance(1e-3, [&](const ToothEdge& e) { r.edges.push_back(e); });
    r.tick_end.push_back(static_cast<std::uint32_t>(r.edges.size()));
    r.signals.push_back(VehicleSignals{sim.sample_yaw_rate(), sim.brake()});
  }
  return r;
}

struct Result {
  double ns_per_edge;
  double mean_call_ns;
  double p99_call_ns;
  double max_call_ns;
  double load;  // fraction of one core in real time
  double edges_per_s;
};

Result replay(const Recording& r) {
  auto det = std::make_unique<Detector>();
  std::vector<double> call_ns;
  call_ns.reserve(r.tick_end.size());
  std::size_t e = 0;
  double total = 0;
  for (std::size_t i = 0; i < r.tick_end.size(); ++i) {
    const auto t0 = Clock::now();
    for (; e < r.tick_end[i]; ++e) det->on_tooth_edge(static_cast<Wheel>(r.edges[e].wheel), r.edges[e].ticks);
    if (i % 10 == 0) det->on_vehicle_signals(r.signals[i]);
    det->process();
    const double ns = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - t0).count());
    call_ns.push_back(ns);
    total += ns;
  }
  std::vector<double> sorted = call_ns;
  std::sort(sorted.begin(), sorted.end());
  Result res{};
  res.ns_per_edge = total / static_cast<double>(r.edges.size());
  res.mean_call_ns = total / static_cast<double>(call_ns.size());
  res.p99_call_ns = sorted[sorted.size() * 99 / 100];
  res.max_call_ns = sorted.back();
  res.load = total / (r.seconds * 1e9);
  res.edges_per_s = static_cast<double>(r.edges.size()) / r.seconds;
  return res;
}

double bench_fft() {
  std::array<float, kFftSize> x{};
  for (std::size_t i = 0; i < kFftSize; ++i) x[i] = static_cast<float>((i * 7919) % 101) - 50.0f;
  std::array<Complex, RealFft<kFftSize>::kBins> out{};
  constexpr int kIters = 200000;
  float sink = 0;
  const auto t0 = Clock::now();
  for (int i = 0; i < kIters; ++i) {
    x[0] = static_cast<float>(i & 7);
    RealFft<kFftSize>::forward(x.data(), out.data());
    sink += out[3].re;
  }
  const double ns = static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - t0).count());
  if (sink == 12345.0f) std::puts("");
  return ns / kIters;
}

}  // namespace

int main() {
  const MemoryReport m = Detector::memory_report();
  std::printf("RAM: sizeof(Detector) = %zu B / %zu B budget (%.0f%%)\n", m.total, m.budget,
              100.0 * static_cast<double>(m.total) / static_cast<double>(m.budget));
  std::printf("     edge queue %zu | tooth tracking %zu | resampler %zu | spectra %zu | fft scratch %zu | radius model %zu\n\n",
              m.edge_queue, m.tooth_tracking, m.resampler, m.spectra, m.fft_scratch, m.radius_model);

  std::printf("256-pt real FFT: %.0f ns\n\n", bench_fft());

  struct Case {
    const char* name;
    Scenario sc;
  };
  const Case cases[] = {{"highway", Scenario::Highway}, {"city", Scenario::City}, {"mixed", Scenario::Mixed}};
  std::printf("%-8s %10s %10s %12s %12s %12s %10s\n", "drive", "edges/s", "ns/edge", "mean task", "p99 task",
              "max task", "CPU load");
  for (const Case& c : cases) {
    const Recording r = record(c.sc, 600.0, 7);
    Result best{};
    for (int rep = 0; rep < 3; ++rep) {
      const Result res = replay(r);
      if (rep == 0 || res.ns_per_edge < best.ns_per_edge) best = res;
    }
    std::printf("%-8s %10.0f %10.1f %10.0f ns %10.0f ns %10.0f ns %9.4f%%\n", c.name, best.edges_per_s, best.ns_per_edge,
                best.mean_call_ns, best.p99_call_ns, best.max_call_ns, 100.0 * best.load);
  }

  // Worst sustained input: 250 km/h, 0.31 m radius, 48 teeth, 4 wheels.
  const double worst_edges = 4.0 * 48.0 * (250.0 / 3.6) / (2.0 * 3.14159265 * 0.31);
  std::printf("\nWorst-case input rate at 250 km/h: %.0f edges/s; queue of %zu covers %.1f ms of task stall.\n",
              worst_edges, kEdgeQueueSize, 1000.0 * static_cast<double>(kEdgeQueueSize) / worst_edges);
  return 0;
}
