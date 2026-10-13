// Embind surface for the browser demo: the same detector + simulator that the
// native tests run, compiled to WebAssembly.
#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <memory>

#include "harness.hpp"

using emscripten::val;
using namespace itpms;
using namespace itpms::sim;

namespace {

val float_array(const float* data, std::size_t n) {
  return val::global("Float32Array").new_(emscripten::typed_memory_view(n, data));
}

class Demo {
 public:
  Demo() : h_(std::make_unique<Harness>(seed_)) {}

  void step(double sim_seconds) { h_->run_for(sim_seconds); }

  void reset(int seed) {
    const Scenario sc = h_->sim().scenario();
    const Road road = h_->sim().road();
    const CatsEyes ce = h_->sim().cats_eyes();
    seed_ = static_cast<std::uint64_t>(seed);
    h_->reset(seed_);
    h_->sim().set_scenario(sc);
    h_->sim().set_road(road);
    h_->sim().set_cats_eyes(ce);
  }

  void set_scenario(int s) { h_->sim().set_scenario(static_cast<Scenario>(s)); }
  void set_road(int r) { h_->sim().set_road(static_cast<Road>(r)); }
  void set_cats_eyes(int m) { h_->sim().set_cats_eyes(static_cast<CatsEyes>(m)); }
  void set_pressure(int w, double frac) {
    h_->sim().set_pressure_fraction(static_cast<std::size_t>(w), frac);
    h_->sim().set_leak(static_cast<std::size_t>(w), 0.0);
  }
  void set_leak(int w, double psi_per_min) { h_->sim().set_leak(static_cast<std::size_t>(w), psi_per_min); }
  void inflate_all() { h_->sim().inflate_all(); }
  void recalibrate() { h_->detector().request_recalibration(); }

  val spectrum(int w) const { return float_array(h_->detector().spectrum(static_cast<Wheel>(w)).data(), kPsdBins); }
  val reference(int w) const {
    return float_array(h_->detector().reference_spectrum(static_cast<Wheel>(w)).data(), kPsdBins);
  }

  val snapshot() const {
    VehicleSim& sim = h_->sim();
    const Status s = h_->detector().status();
    val o = val::object();
    o.set("t", sim.time_s());
    o.set("speedTrue", sim.speed_mps() * 3.6);
    o.set("speed", s.speed_kmh);
    o.set("accel", s.accel_mps2);
    o.set("latAccel", sim.speed_mps() * sim.speed_mps() * sim.curvature());
    o.set("scenario", static_cast<int>(sim.scenario()));
    o.set("road", static_cast<int>(sim.road()));
    o.set("monitoring", s.state == SystemState::Monitoring);
    o.set("calProgress", s.calibration_progress);
    o.set("allLow", s.all_low);
    o.set("radiusSamples", s.radius_samples);
    o.set("onStuds", sim.on_studs());
    o.set("studHits", sim.stud_hits());
    o.set("impulseFrames", s.impulse_frames);

    val wheels = val::array();
    for (std::size_t w = 0; w < kWheels; ++w) {
      const WheelStatus& ws = s.wheel[w];
      val x = val::object();
      x.set("frac", sim.pressure_fraction(w));
      x.set("psi", sim.pressure_psi(w));
      x.set("leak", sim.leak(w));
      x.set("fTrue", sim.resonance_hz(w));
      x.set("loss", ws.loss);
      x.set("lossR", ws.loss_radius);
      x.set("lossF", ws.loss_freq);
      x.set("f", ws.freq_hz);
      x.set("fRef", ws.freq_ref_hz);
      x.set("warn", ws.warning);
      x.set("frames", s.freq_frames[w]);
      wheels.call<void>("push", x);
    }
    o.set("wheels", wheels);

    val residual = val::array();
    val noise = val::array();
    for (std::size_t j = 0; j < kIndicators; ++j) {
      residual.call<void>("push", s.residual[j]);
      noise.call<void>("push", s.noise[j]);
    }
    o.set("residual", residual);
    o.set("noise", noise);

    val gates = val::array();
    for (std::uint32_t g : s.gates) gates.call<void>("push", g);
    o.set("gates", gates);

    const PerfStats& p = h_->perf();
    val perf = val::object();
    perf.set("detectorNs", p.detector_ns);
    perf.set("simSeconds", p.sim_seconds);
    perf.set("edges", static_cast<double>(p.edges));
    perf.set("dropped", s.edges_dropped);
    o.set("perf", perf);
    return o;
  }

  static val memory() {
    const MemoryReport m = Detector::memory_report();
    val o = val::object();
    o.set("total", m.total);
    o.set("budget", m.budget);
    o.set("queue", m.edge_queue);
    o.set("teeth", m.tooth_tracking);
    o.set("resampler", m.resampler);
    o.set("spectra", m.spectra);
    o.set("fft", m.fft_scratch);
    o.set("model", m.radius_model);
    o.set("psdLoHz", static_cast<double>(kPsdLoBin) * kBinHz);
    o.set("binHz", kBinHz);
    o.set("bins", kPsdBins);
    return o;
  }

 private:
  std::uint64_t seed_ = 1;
  std::unique_ptr<Harness> h_;
};

}  // namespace

EMSCRIPTEN_BINDINGS(itpms) {
  emscripten::class_<Demo>("Demo")
      .constructor<>()
      .function("step", &Demo::step)
      .function("reset", &Demo::reset)
      .function("setScenario", &Demo::set_scenario)
      .function("setRoad", &Demo::set_road)
      .function("setCatsEyes", &Demo::set_cats_eyes)
      .function("setPressure", &Demo::set_pressure)
      .function("setLeak", &Demo::set_leak)
      .function("inflateAll", &Demo::inflate_all)
      .function("recalibrate", &Demo::recalibrate)
      .function("spectrum", &Demo::spectrum)
      .function("reference", &Demo::reference)
      .function("snapshot", &Demo::snapshot)
      .class_function("memory", &Demo::memory);
}
