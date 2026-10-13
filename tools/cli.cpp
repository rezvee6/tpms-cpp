// Headless scenario runner: drive, change tire pressures at given times and
// print what the detector believes.
//
//   itpms_cli --scenario highway --minutes 20 --set 9:FL:0.7 --leak 12:RR:1.5
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "harness.hpp"

using namespace itpms;
using namespace itpms::sim;

namespace {

struct Event {
  double minute;
  int wheel;  // -1 = all
  bool leak;
  double value;
};

const char* kWheelNames[] = {"FL", "FR", "RL", "RR"};

int parse_wheel(const std::string& s) {
  for (int i = 0; i < 4; ++i)
    if (s == kWheelNames[i]) return i;
  if (s == "ALL") return -1;
  std::fprintf(stderr, "unknown wheel '%s'\n", s.c_str());
  std::exit(2);
}

Event parse_event(const char* arg, bool leak) {
  char wheel[8] = {};
  Event e{0, 0, leak, 0};
  if (std::sscanf(arg, "%lf:%7[A-Z]:%lf", &e.minute, wheel, &e.value) != 3) {
    std::fprintf(stderr, "bad event '%s' (want MIN:WHEEL:VALUE)\n", arg);
    std::exit(2);
  }
  e.wheel = parse_wheel(wheel);
  return e;
}

void print_status(Harness& h) {
  const Status s = h.detector().status();
  VehicleSim& sim = h.sim();
  std::printf("t=%6.1fmin v=%5.1f %-11s", sim.time_s() / 60.0, s.speed_kmh,
              s.state == SystemState::Calibrating ? "CALIBRATING" : "MONITORING");
  if (s.state == SystemState::Calibrating) std::printf(" %3.0f%%", 100.0f * s.calibration_progress);
  std::printf(" |");
  for (std::size_t w = 0; w < kWheels; ++w) {
    const WheelStatus& ws = s.wheel[w];
    std::printf(" %s true=%4.0f%% est=%5.1f%% (r%5.1f f%5.1f %4.1fHz/%4.1f)%s", kWheelNames[w],
                100.0 * (1.0 - sim.pressure_fraction(w)), 100.0f * ws.loss, 100.0f * ws.loss_radius,
                100.0f * ws.loss_freq, ws.freq_hz, ws.freq_ref_hz, ws.warning ? " WARN" : "");
  }
  std::printf("%s\n", s.all_low ? " ALL-LOW" : "");
}

}  // namespace

int main(int argc, char** argv) {
  Scenario scenario = Scenario::Highway;
  Road road = Road::Normal;
  CatsEyes cats_eyes = CatsEyes::Off;
  double minutes = 20.0;
  double every = 60.0;
  std::uint64_t seed = 1;
  std::vector<Event> events;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    const char* v = i + 1 < argc ? argv[i + 1] : "";
    if (a == "--scenario") {
      const std::string s = argv[++i];
      scenario = s == "city" ? Scenario::City : s == "country" ? Scenario::Country : s == "mixed" ? Scenario::Mixed
                                                                                                  : Scenario::Highway;
    } else if (a == "--road") {
      const std::string s = argv[++i];
      road = s == "smooth" ? Road::Smooth : s == "rough" ? Road::Rough : Road::Normal;
    } else if (a == "--cats-eyes") {
      const std::string m = argv[++i];
      cats_eyes = m == "straddle" ? CatsEyes::Straddle : m == "off" ? CatsEyes::Off : CatsEyes::LaneChanges;
    } else if (a == "--minutes") {
      minutes = std::atof(v), ++i;
    } else if (a == "--every") {
      every = std::atof(v), ++i;
    } else if (a == "--seed") {
      seed = std::strtoull(v, nullptr, 10), ++i;
    } else if (a == "--set") {
      events.push_back(parse_event(v, false)), ++i;
    } else if (a == "--leak") {
      events.push_back(parse_event(v, true)), ++i;
    } else {
      std::fprintf(stderr, "usage: %s [--scenario highway|country|city|mixed] [--road smooth|normal|rough] [--cats-eyes lanes|straddle]\n"
                           "          [--minutes N] [--every SEC] [--seed N] [--set MIN:WHEEL:FRAC] [--leak MIN:WHEEL:PSI_PER_MIN]\n",
                   argv[0]);
      return 2;
    }
  }

  auto h = std::make_unique<Harness>(seed);
  h->sim().set_scenario(scenario);
  h->sim().set_road(road);
  h->sim().set_cats_eyes(cats_eyes);

  const auto steps = static_cast<int>(minutes * 60.0 / every);
  for (int i = 0; i < steps; ++i) {
    const double t0 = i * every / 60.0;
    for (const Event& e : events) {
      if (e.minute < t0 || e.minute >= t0 + every / 60.0) continue;
      for (int w = 0; w < 4; ++w) {
        if (e.wheel != -1 && e.wheel != w) continue;
        if (e.leak) h->sim().set_leak(static_cast<std::size_t>(w), e.value);
        else h->sim().set_pressure_fraction(static_cast<std::size_t>(w), e.value);
      }
    }
    h->run_for(every);
    print_status(*h);
  }

  const PerfStats& p = h->perf();
  const Status s = h->detector().status();
  std::printf("\nedges=%llu dropped=%u  detector: %.1f ns/edge, %.4f%% of one core in real time\n",
              static_cast<unsigned long long>(p.edges), s.edges_dropped, p.detector_ns / static_cast<double>(p.edges),
              100.0 * p.detector_ns / (p.sim_seconds * 1e9));
  std::printf("FFT frames with impulses clipped: %u\n", s.impulse_frames);
  std::printf("gates: ok=%u speed=%u brake=%u accel=%u corner=%u spread=%u\n", s.gates[0], s.gates[1], s.gates[2],
              s.gates[3], s.gates[4], s.gates[5]);
  std::printf("radius residual (x1e-4): diag %.2f side %.2f axle %.2f  noise %.2f %.2f %.2f\n", 1e4f * s.residual[0],
              1e4f * s.residual[1], 1e4f * s.residual[2], 1e4f * s.noise[0], 1e4f * s.noise[1], 1e4f * s.noise[2]);
  return 0;
}
