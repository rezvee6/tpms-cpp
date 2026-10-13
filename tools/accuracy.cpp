// Accuracy sweep: many randomised cars (tire spread, sensitivity, resonance
// all differ per seed) x every wheel x several loss levels, with and without
// cat's eyes. Prints markdown tables used in docs/ACCURACY.md.
//
//   itpms_accuracy [seeds_per_case]
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <string>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "harness.hpp"

using namespace itpms;
using namespace itpms::sim;

namespace {

struct Job {
  bool robust;  // median pre-filter + impulse clipping
  CatsEyes cats_eyes;
  double loss;  // 0 = no fault; applies to `wheel`, or all four if wheel == 4
  int wheel;
  std::uint64_t seed;
};

struct Outcome {
  bool calibrated = false;
  double est = 0;            // mean estimate of the deflated wheel(s), last 3 min
  double others_max = 0;     // max |estimate| on healthy wheels, last 3 min
  bool warned = false;       // deflated wheel(s) warned at the end
  bool false_alarm = false;  // any healthy wheel warned after deflation
  double latency_min = -1;   // time to first warning
};

Outcome run(const Job& j) {
  Outcome o;
  DetectorConfig cfg;
  cfg.median_prefilter = j.robust;
  cfg.impulse_clip_sigma = j.robust ? cfg.impulse_clip_sigma : 0.0f;
  auto h = std::make_unique<Harness>(j.seed, cfg);
  h->sim().set_scenario(j.seed % 2 ? Scenario::Highway : Scenario::Country);
  h->sim().set_cats_eyes(j.cats_eyes);

  for (int i = 0; i < 120 && h->detector().status().state != SystemState::Monitoring; ++i) h->run_for(10.0);
  if (h->detector().status().state != SystemState::Monitoring) return o;
  o.calibrated = true;

  const auto low = [&](int w) { return j.loss > 0 && (j.wheel == 4 || j.wheel == w); };
  for (int w = 0; w < 4; ++w)
    if (low(w)) h->sim().set_pressure_fraction(static_cast<std::size_t>(w), 1.0 - j.loss);

  constexpr int kSteps = 10 * 12;  // 10 min in 5 s steps
  double sum = 0;
  int n = 0;
  for (int i = 0; i < kSteps; ++i) {
    h->run_for(5.0);
    const Status s = h->detector().status();
    bool all_low_warned = true;
    for (int w = 0; w < 4; ++w) {
      if (low(w)) {
        all_low_warned = all_low_warned && s.wheel[w].warning;
        if (i >= kSteps - 36) sum += s.wheel[w].loss, ++n;
      } else {
        if (s.wheel[w].warning) o.false_alarm = true;
        if (i >= kSteps - 36) o.others_max = std::max(o.others_max, std::fabs(double(s.wheel[w].loss)));
      }
    }
    if (j.loss > 0 && all_low_warned && o.latency_min < 0) o.latency_min = (i + 1) * 5.0 / 60.0;
    if (i == kSteps - 1) o.warned = j.loss > 0 && all_low_warned;
  }
  o.est = n ? sum / n : 0;
  return o;
}

struct Stats {
  std::vector<double> err, others, latency;
  int runs = 0, warned = 0, false_alarms = 0, uncalibrated = 0;
};

double mean(const std::vector<double>& v) {
  double s = 0;
  for (double x : v) s += x;
  return v.empty() ? 0 : s / static_cast<double>(v.size());
}
double stdev(const std::vector<double>& v) {
  const double m = mean(v);
  double s = 0;
  for (double x : v) s += (x - m) * (x - m);
  return v.size() > 1 ? std::sqrt(s / static_cast<double>(v.size() - 1)) : 0;
}
double max_abs(const std::vector<double>& v) {
  double m = 0;
  for (double x : v) m = std::max(m, std::fabs(x));
  return m;
}


void collect(const std::vector<Job>& jobs, const std::vector<Outcome>& out, Stats& st,
             const std::function<bool(const Job&)>& pick) {
  for (std::size_t i = 0; i < jobs.size(); ++i) {
    const Job& j = jobs[i];
    if (!pick(j)) continue;
    const Outcome& o = out[i];
    if (!o.calibrated) { ++st.uncalibrated; continue; }
    ++st.runs;
    if (j.loss > 0) st.err.push_back(o.est - j.loss);
    st.others.push_back(o.others_max);
    st.warned += o.warned;
    st.false_alarms += o.false_alarm;
    if (o.latency_min >= 0) st.latency.push_back(o.latency_min);
  }
  std::sort(st.latency.begin(), st.latency.end());
}

std::string median_latency(const Stats& st) {
  if (st.latency.empty()) return "–";
  char b[32];
  std::snprintf(b, sizeof b, "%.1f min", st.latency[st.latency.size() / 2]);
  return b;
}

}  // namespace

int main(int argc, char** argv) {
  const int seeds = argc > 1 ? std::atoi(argv[1]) : 6;
  const double losses[] = {0.0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.40};
  const CatsEyes modes[] = {CatsEyes::Off, CatsEyes::LaneChanges, CatsEyes::Straddle};
  const char* mode_names[] = {"plain road", "cat's eyes, lane changes", "cat's eyes, straddling all trip"};

  std::vector<Job> jobs;
  for (bool robust : {false, true})
    for (CatsEyes ce : modes)
      for (double loss : losses)
        for (int w = 0; w < 5; ++w) {
          if (loss == 0 && w > 0) continue;
          const int reps = loss == 0 ? 4 * seeds : seeds;  // more no-fault drives for the false-alarm check
          for (int s = 0; s < reps; ++s)
            jobs.push_back({robust, ce, loss, w,
                            static_cast<std::uint64_t>(1000 + s * 7 + w * 101 + static_cast<int>(loss * 1000))});
        }

  std::vector<Outcome> out(jobs.size());
  std::atomic<std::size_t> next{0};
  std::vector<std::thread> pool;
  for (unsigned t = 0; t < std::max(1u, std::thread::hardware_concurrency()); ++t)
    pool.emplace_back([&] {
      for (std::size_t i; (i = next++) < jobs.size();) out[i] = run(jobs[i]);
    });
  for (auto& t : pool) t.join();
  std::printf("<!-- %zu simulated drives, %d seeds per case -->\n", jobs.size(), seeds);

  // Table 1: accuracy by loss level (robust detector, plain road).
  std::printf("\n### Accuracy by loss level (plain road)\n\n");
  std::printf("| True loss | Tires | Runs | Mean error | Std dev | Worst error | Warned | Median time to warn |\n");
  std::printf("|---|---|---|---|---|---|---|---|\n");
  for (double loss : losses) {
    if (loss == 0) continue;
    for (int group : {0, 4}) {
      Stats st;
      collect(jobs, out, st, [&](const Job& j) {
        return j.robust && j.cats_eyes == CatsEyes::Off && j.loss == loss && (group == 4) == (j.wheel == 4);
      });
      std::printf("| %.0f%% | %s | %d | %+.1f pp | %.1f pp | %.1f pp | %d/%d | %s |\n", 100 * loss,
                  group == 4 ? "all four" : "one", st.runs, 100 * mean(st.err), 100 * stdev(st.err),
                  100 * max_abs(st.err), st.warned, st.runs, median_latency(st).c_str());
    }
  }

  // Table 2: robustness to cat's eyes, with and without the impulse defences.
  std::printf("\n### Cat's eyes: impulse defences off vs on\n\n");
  std::printf("| Road | Defences | Estimate std dev | Worst error | Detected (loss >= 20%%) | False alarms | Healthy-tire noise (max) |\n");
  std::printf("|---|---|---|---|---|---|---|\n");
  for (CatsEyes ce : modes)
    for (bool robust : {false, true}) {
      Stats all, big, healthy;
      collect(jobs, out, all, [&](const Job& j) { return j.robust == robust && j.cats_eyes == ce && j.loss > 0; });
      collect(jobs, out, big, [&](const Job& j) { return j.robust == robust && j.cats_eyes == ce && j.loss >= 0.2; });
      collect(jobs, out, healthy, [&](const Job& j) { return j.robust == robust && j.cats_eyes == ce; });
      std::printf("| %s | %s | %.1f pp | %.1f pp | %d/%d | %d/%d drives | %.1f%% |\n", mode_names[static_cast<int>(ce)],
                  robust ? "on" : "off", 100 * stdev(all.err), 100 * max_abs(all.err), big.warned, big.runs,
                  healthy.false_alarms, healthy.runs, 100 * max_abs(healthy.others));
    }

  Stats floor;
  collect(jobs, out, floor, [](const Job& j) { return j.robust; });
  std::printf("\nHealthy-tire estimate never exceeded %.1f%% in %d robust drives (zero-false-alarm floor).\n",
              100 * max_abs(floor.others), floor.runs);
  return 0;
}
