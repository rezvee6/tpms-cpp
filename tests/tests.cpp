// Unit + scenario tests. Zero dependencies: a tiny CHECK macro and a runner.
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <memory>
#include <new>
#include <random>
#include <string>
#include <vector>

#include "harness.hpp"
#include "itpms/detector.hpp"
#include "itpms/fft.hpp"
#include "itpms/spsc_ring.hpp"

using namespace itpms;
using namespace itpms::sim;

// --- Heap tracking: proves the detector never allocates -------------------
static std::atomic<bool> g_track{false};
static std::atomic<long> g_allocs{0};

void* operator new(std::size_t n) {
  if (g_track.load(std::memory_order_relaxed)) g_allocs.fetch_add(1);
  if (void* p = std::malloc(n ? n : 1)) return p;
  throw std::bad_alloc();
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }

// --- Minimal test framework ------------------------------------------------
static int g_failures = 0;
#define CHECK(cond, ...)                                                   \
  do {                                                                     \
    if (!(cond)) {                                                         \
      ++g_failures;                                                        \
      std::printf("    FAIL %s:%d: %s  ", __FILE__, __LINE__, #cond);     \
      std::printf(__VA_ARGS__);                                            \
      std::printf("\n");                                                   \
    }                                                                      \
  } while (0)

struct TestCase {
  const char* name;
  std::function<void()> fn;
};

static const char* kNames[] = {"FL", "FR", "RL", "RR"};

// --- Unit tests --------------------------------------------------------------
static void test_ct_trig() {
  double worst = 0;
  for (int i = -2000; i <= 2000; ++i) {
    const double x = i * 0.01;
    worst = std::max(worst, std::fabs(ct::sin(x) - std::sin(x)));
    worst = std::max(worst, std::fabs(ct::cos(x) - std::cos(x)));
  }
  CHECK(worst < 1e-10, "max error %.3g", worst);
}

static void test_real_fft_matches_dft() {
  constexpr std::size_t N = 256;
  std::mt19937 rng(42);
  std::normal_distribution<float> dist;
  std::array<float, N> x{};
  for (float& v : x) v = dist(rng);
  std::array<Complex, N / 2 + 1> out{};
  RealFft<N>::forward(x.data(), out.data());

  double worst = 0, scale = 0;
  for (std::size_t k = 0; k <= N / 2; ++k) {
    double re = 0, im = 0;
    for (std::size_t n = 0; n < N; ++n) {
      const double a = -2.0 * M_PI * double(k) * double(n) / double(N);
      re += x[n] * std::cos(a);
      im += x[n] * std::sin(a);
    }
    worst = std::max(worst, std::hypot(re - out[k].re, im - out[k].im));
    scale = std::max(scale, std::hypot(re, im));
  }
  CHECK(worst / scale < 1e-5, "relative error %.3g", worst / scale);
}

static void test_spsc_ring() {
  SpscRing<int, 8> q;
  for (int i = 0; i < 8; ++i) CHECK(q.push(i), "push %d", i);
  CHECK(!q.push(99), "push into full ring must fail");
  int v = -1;
  for (int i = 0; i < 8; ++i) {
    CHECK(q.pop(v) && v == i, "pop %d got %d", i, v);
  }
  CHECK(!q.pop(v), "pop from empty ring must fail");
  // Many wraps of the free-running indices.
  for (int i = 0; i < 100000; ++i) {
    q.push(i);
    q.pop(v);
    if (v != i) {
      CHECK(v == i, "wrap at %d", i);
      break;
    }
  }
}

static void test_memory_budget() {
  const MemoryReport r = Detector::memory_report();
  std::printf("    sizeof(Detector) = %zu B of %zu B budget (queue %zu, teeth %zu, resampler %zu, spectra %zu, fft %zu)\n",
              r.total, r.budget, r.edge_queue, r.tooth_tracking, r.resampler, r.spectra, r.fft_scratch);
  CHECK(r.total <= r.budget, "over budget");
}

static void test_no_heap_in_detector() {
  // Record 60 s of edges first (the simulator may allocate), then replay.
  VehicleSim sim(3);
  std::vector<ToothEdge> edges;
  edges.reserve(1 << 20);
  sim.advance(60.0, [&](const ToothEdge& e) { edges.push_back(e); });

  auto det = std::make_unique<Detector>();
  g_allocs = 0;
  g_track = true;
  for (std::size_t i = 0; i < edges.size(); ++i) {
    det->on_tooth_edge(static_cast<Wheel>(edges[i].wheel), edges[i].ticks);
    if (i % 4 == 0) det->process();
  }
  det->process();
  volatile auto s = det->status();
  (void)s;
  g_track = false;
  CHECK(g_allocs == 0, "detector allocated %ld times", g_allocs.load());
  CHECK(det->status().edges_processed == edges.size(), "processed %u of %zu", det->status().edges_processed,
        edges.size());
}

static void test_queue_overflow_is_counted() {
  auto det = std::make_unique<Detector>();
  for (std::uint32_t i = 0; i < kEdgeQueueSize + 10; ++i) det->on_tooth_edge(FL, i * 1000);
  CHECK(det->status().edges_dropped == 10, "dropped %u", det->status().edges_dropped);
}

// --- Scenario tests ----------------------------------------------------------
struct Run {
  std::unique_ptr<Harness> h;
  bool false_alarm = false;
  double first_warn_min[4] = {-1, -1, -1, -1};
  double all_low_min = -1;
};

// Runs `minutes`, sampling the detector every 10 s. `deflated` marks wheels
// that are (or will be) low, so warnings on other wheels count as false alarms.
static Run drive(std::uint64_t seed, Scenario sc, double minutes, const std::function<void(Harness&, double)>& events,
                 std::array<bool, 4> deflated = {}) {
  Run r;
  r.h = std::make_unique<Harness>(seed);
  r.h->sim().set_scenario(sc);
  for (double t = 0; t < minutes * 60.0; t += 10.0) {
    events(*r.h, t / 60.0);
    r.h->run_for(10.0);
    const Status s = r.h->detector().status();
    const double now = (t + 10.0) / 60.0;
    for (int w = 0; w < 4; ++w) {
      if (!s.wheel[w].warning) continue;
      if (r.first_warn_min[w] < 0) r.first_warn_min[w] = now;
      if (!deflated[w]) r.false_alarm = true;
    }
    if (s.all_low && r.all_low_min < 0) r.all_low_min = now;
  }
  return r;
}

static void test_no_false_alarm(Scenario sc, std::uint64_t seed, Road road) {
  Run r = drive(seed, sc, 40.0, [&](Harness& h, double t) {
    if (t == 0) h.sim().set_road(road);
  });
  CHECK(!r.false_alarm, "false alarm, first warnings %.1f %.1f %.1f %.1f", r.first_warn_min[0], r.first_warn_min[1],
        r.first_warn_min[2], r.first_warn_min[3]);
  CHECK(r.h->detector().status().state == SystemState::Monitoring, "never finished calibrating");
}

static void test_single_wheel(Scenario sc, int wheel, std::uint64_t seed) {
  // Deflate at minute 12, or once calibration completes if city driving made
  // it slower (deflating during calibration would teach it the wrong baseline).
  double at = -1;
  std::array<bool, 4> low{};
  low[wheel] = true;
  Run r = drive(
      seed, sc, 30.0,
      [&](Harness& h, double t) {
        if (at < 0 && t >= 12.0 && h.detector().status().state == SystemState::Monitoring) {
          at = t;
          h.sim().set_pressure_fraction(static_cast<std::size_t>(wheel), 0.7);
        }
      },
      low);
  const double kAt = at;
  CHECK(at > 0, "calibration never finished");
  const double latency = r.first_warn_min[wheel] - kAt;
  std::printf("    %s -30%%: warned after %.1f min\n", kNames[wheel], latency);
  CHECK(!r.false_alarm, "wrong wheel warned");
  CHECK(r.first_warn_min[wheel] > kAt && latency <= 5.0, "latency %.1f min", latency);
}

static void test_all_four_low() {
  constexpr double kAt = 12.0;
  Run r = drive(
      11, Scenario::Highway, 22.0,
      [&](Harness& h, double t) {
        if (t == kAt)
          for (std::size_t w = 0; w < 4; ++w) h.sim().set_pressure_fraction(w, 0.7);
      },
      {true, true, true, true});
  std::printf("    all four -30%%: ALL-LOW after %.1f min\n", r.all_low_min - kAt);
  CHECK(r.all_low_min > kAt && r.all_low_min - kAt <= 5.0, "all-low at %.1f min", r.all_low_min);
}

static void test_slow_leak() {
  // 0.7 psi/min from 35 psi: crosses 20 % after 10 min, 30 % after 15 min.
  constexpr double kAt = 12.0;
  Run r = drive(
      5, Scenario::Mixed, 32.0,
      [&](Harness& h, double t) {
        if (t == kAt) h.sim().set_leak(RL, 0.7);
      },
      {false, false, true, false});
  const double lost = 0.7 * (r.first_warn_min[RL] - kAt) / 35.0;
  std::printf("    RL slow leak: warned at %.0f%% true loss\n", 100 * lost);
  CHECK(!r.false_alarm, "wrong wheel warned");
  CHECK(r.first_warn_min[RL] > kAt && lost <= 0.30, "warned at %.0f%% loss", 100 * lost);
}

static void test_recalibration_clears() {
  auto h = std::make_unique<Harness>(9);
  h->run_for(9 * 60.0);
  h->sim().set_pressure_fraction(FR, 0.65);
  h->run_for(4 * 60.0);
  CHECK(h->detector().status().wheel[FR].warning, "FR should be warned before reset");
  h->sim().inflate_all();
  h->detector().request_recalibration();
  h->run_for(12 * 60.0);
  const Status s = h->detector().status();
  CHECK(s.state == SystemState::Monitoring, "not re-calibrated");
  for (int w = 0; w < 4; ++w) CHECK(!s.wheel[w].warning, "%s still warned", kNames[w]);
}

int main() {
  std::vector<TestCase> tests = {
      {"constexpr trig accuracy", test_ct_trig},
      {"real FFT == naive DFT", test_real_fft_matches_dft},
      {"SPSC ring", test_spsc_ring},
      {"RAM budget", test_memory_budget},
      {"zero heap allocations in detector", test_no_heap_in_detector},
      {"queue overflow counted", test_queue_overflow_is_counted},
      {"no false alarm: highway, normal road", [] { test_no_false_alarm(Scenario::Highway, 21, Road::Normal); }},
      {"no false alarm: mixed, rough road", [] { test_no_false_alarm(Scenario::Mixed, 22, Road::Rough); }},
      {"no false alarm: city, smooth road", [] { test_no_false_alarm(Scenario::City, 23, Road::Smooth); }},
      {"single wheel FL (highway)", [] { test_single_wheel(Scenario::Highway, FL, 31); }},
      {"single wheel FR (country)", [] { test_single_wheel(Scenario::Country, FR, 32); }},
      {"single wheel RL (mixed)", [] { test_single_wheel(Scenario::Mixed, RL, 33); }},
      {"single wheel RR (city)", [] { test_single_wheel(Scenario::City, RR, 34); }},
      {"all four tires low", test_all_four_low},
      {"slow leak", test_slow_leak},
      {"recalibration clears warnings", test_recalibration_clears},
  };
  for (const auto& t : tests) {
    const int before = g_failures;
    std::printf("[ RUN  ] %s\n", t.name);
    t.fn();
    std::printf("[ %s ] %s\n", g_failures == before ? " OK " : "FAIL", t.name);
  }
  std::printf("\n%s: %d failure(s)\n", g_failures ? "FAILED" : "PASSED", g_failures);
  return g_failures ? 1 : 0;
}
