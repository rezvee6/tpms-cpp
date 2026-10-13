// Vehicle + tire physics simulator that produces what a real ECU would see:
// raw ABS tooth-edge timestamps from a 10 MHz capture timer, a noisy yaw-rate
// sensor and a brake switch. It is the "ground truth" side of the demo and is
// deliberately not constrained like the detector (double precision, more RAM).
//
// Modelled effects: driver/road profiles, cornering kinematics, drive and
// brake slip vs. load, aero drag, per-tire manufacturing radius spread,
// pressure-dependent rolling radius, sidewall torsional resonance excited by
// road roughness, per-wheel road noise, tone-ring tooth spacing errors, timer
// quantisation and 32-bit timer wrap.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "itpms/config.hpp"

namespace itpms::sim {

enum class Scenario : std::uint8_t { Highway, Country, City, Mixed };
enum class Road : std::uint8_t { Smooth, Normal, Rough };

struct ToothEdge {
  std::uint32_t ticks;
  std::uint8_t wheel;
};

// SplitMix64 + Box-Muller: tiny, fast and reproducible across platforms.
class Rng {
 public:
  explicit Rng(std::uint64_t seed) : s_(seed) {}
  std::uint64_t next();
  double uniform() { return static_cast<double>(next() >> 11) * 0x1.0p-53; }
  double uniform(double a, double b) { return a + (b - a) * uniform(); }
  double gauss();

 private:
  std::uint64_t s_;
  double spare_ = 0.0;
  bool has_spare_ = false;
};

// Hidden per-tire truth; the detector only knows nominal values.
struct TireParams {
  double nominal_psi = 35.0;
  double radius_offset = 0.0;  // manufacturing spread of rolling radius (fraction)
  double radius_sensitivity = 0.010;
  double f0_hz = 42.0;
  double freq_exponent = 0.25;
  double zeta = 0.12;
};

class VehicleSim {
 public:
  static constexpr double kStep = 1.0 / 4000.0;

  explicit VehicleSim(std::uint64_t seed = 1);
  void reset(std::uint64_t seed);

  void set_scenario(Scenario s) { scenario_ = s; seg_left_ = 0.0; }
  void set_road(Road r) { road_ = r; }
  void set_pressure(std::size_t w, double psi) { pressure_[w] = psi; }
  void set_pressure_fraction(std::size_t w, double frac) { pressure_[w] = frac * tire_[w].nominal_psi; }
  void set_leak(std::size_t w, double psi_per_min) { leak_[w] = psi_per_min; }
  void inflate_all();

  // Advance simulated time; `sink(const ToothEdge&)` receives edges in time order.
  template <class Sink>
  void advance(double seconds, Sink&& sink) {
    carry_ += seconds;
    while (carry_ >= kStep) {
      carry_ -= kStep;
      step();
      for (std::size_t i = 0; i < n_edges_; ++i) sink(edges_[i]);
    }
  }

  // Sensors (CAN side).
  float sample_yaw_rate();
  bool brake() const { return accel_ < -0.7; }

  // Ground truth.
  double time_s() const { return t_; }
  double speed_mps() const { return v_; }
  double accel_mps2() const { return accel_; }
  double curvature() const { return kappa_; }
  double pressure_psi(std::size_t w) const { return pressure_[w]; }
  double pressure_fraction(std::size_t w) const { return pressure_[w] / tire_[w].nominal_psi; }
  double leak(std::size_t w) const { return leak_[w]; }
  double resonance_hz(std::size_t w) const;
  const TireParams& tire(std::size_t w) const { return tire_[w]; }
  Scenario scenario() const { return scenario_; }
  Road road() const { return road_; }

 private:
  struct WheelState {
    double phi = 0.0;        // rim angle within the current revolution
    double next_edge = 0.0;  // angle of the next tooth edge
    std::uint32_t tooth = 0;
    double tors_x = 0.0;     // torsional deflection
    double tors_v = 0.0;     // rim speed perturbation (rad/s)
    double road_eps = 0.0;   // slowly varying path-length noise
    std::array<double, kTeeth> tooth_err{};  // edge position error, fraction of pitch
  };

  void step();
  void update_driver(double dt);
  void new_speed_segment();
  void new_curve_segment();
  Scenario active_profile() const;
  double road_gain() const;

  Rng rng_{1};
  Scenario scenario_ = Scenario::Highway;
  Road road_ = Road::Normal;
  std::array<TireParams, kWheels> tire_{};
  std::array<WheelState, kWheels> wheel_{};
  std::array<double, kWheels> pressure_{};
  std::array<double, kWheels> leak_{};

  double t_ = 0.0;
  double carry_ = 0.0;
  double v_ = 0.0;
  double accel_ = 0.0;
  double kappa_ = 0.0;
  double kappa_target_ = 0.0;
  double v_target_ = 0.0;
  double seg_left_ = 0.0;
  double curve_left_ = 0.0;
  double yaw_bias_ = 0.0;
  std::uint32_t tick_offset_ = 0;

  std::array<ToothEdge, 4 * kWheels> edges_{};
  std::size_t n_edges_ = 0;
};

}  // namespace itpms::sim
