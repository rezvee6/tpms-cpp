#include "vehicle_sim.hpp"

#include <algorithm>
#include <cmath>

namespace itpms::sim {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kG = 9.81;
constexpr double kMass = 1500.0;
constexpr double kCdA = 0.65;
constexpr double kRho = 1.2;
constexpr double kCrr = 0.011;
constexpr double kWheelbase = 2.7;
constexpr double kTrack = 1.6;
constexpr double kFrontWeight = 0.6;
constexpr double kSlipStiffness = 18.0;  // longitudinal force / (Fz * slip)
constexpr double kRadius = 0.31;
constexpr double kPitch = 2.0 * kPi / kTeeth;
constexpr double kMaxLatAccel = 3.0;

struct Profile {
  double v_lo, v_hi;          // km/h
  double seg_lo, seg_hi;      // s per speed segment
  double r_lo, r_hi;          // curve radius, m
  double curve_prob;
  double curve_lo, curve_hi;  // s per curve
  double stop_prob;
  double kappa_tau;           // s, steering smoothness
};

constexpr Profile kHighway{95, 130, 40, 120, 700, 2500, 0.35, 10, 40, 0.0, 2.0};
constexpr Profile kCountry{60, 95, 20, 60, 150, 500, 0.5, 5, 15, 0.0, 1.2};
constexpr Profile kCity{25, 50, 10, 40, 20, 60, 0.25, 3, 6, 0.3, 0.6};

const Profile& profile_for(Scenario s) {
  switch (s) {
    case Scenario::Country: return kCountry;
    case Scenario::City: return kCity;
    default: return kHighway;
  }
}

}  // namespace

std::uint64_t Rng::next() {
  std::uint64_t z = (s_ += 0x9E3779B97F4A7C15ull);
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
  return z ^ (z >> 31);
}

double Rng::gauss() {
  if (has_spare_) {
    has_spare_ = false;
    return spare_;
  }
  const double u1 = std::max(uniform(), 1e-300);
  const double u2 = uniform();
  const double r = std::sqrt(-2.0 * std::log(u1));
  spare_ = r * std::sin(2.0 * kPi * u2);
  has_spare_ = true;
  return r * std::cos(2.0 * kPi * u2);
}

VehicleSim::VehicleSim(std::uint64_t seed) { reset(seed); }

void VehicleSim::reset(std::uint64_t seed) {
  rng_ = Rng(seed);
  for (std::size_t w = 0; w < kWheels; ++w) {
    TireParams& tp = tire_[w];
    tp = TireParams{};
    tp.radius_offset = 0.0015 * rng_.gauss();
    tp.radius_sensitivity *= rng_.uniform(0.9, 1.1);
    tp.f0_hz = rng_.uniform(40.0, 45.0);
    tp.freq_exponent *= rng_.uniform(0.9, 1.1);
    tp.zeta = rng_.uniform(0.10, 0.14);

    WheelState& ws = wheel_[w];
    ws = WheelState{};
    for (double& e : ws.tooth_err) e = 0.008 * rng_.gauss();
    ws.phi = rng_.uniform(0.0, kPitch);
    ws.next_edge = (1.0 + ws.tooth_err[1]) * kPitch;
    ws.tooth = 1;

    pressure_[w] = tp.nominal_psi;
    leak_[w] = 0.0;
  }
  t_ = 0.0;
  carry_ = 0.0;
  kappa_ = kappa_target_ = 0.0;
  curve_left_ = 0.0;
  yaw_bias_ = 0.002 * rng_.gauss();
  // Start 20 s before the 32-bit capture timer wraps, so wrap handling is
  // exercised in every run.
  tick_offset_ = UINT32_MAX - 20u * kTimerHz;
  new_speed_segment();
  v_ = v_target_ > 0.0 ? v_target_ : 30.0 / 3.6;
  accel_ = 0.0;
}

void VehicleSim::inflate_all() {
  for (std::size_t w = 0; w < kWheels; ++w) {
    pressure_[w] = tire_[w].nominal_psi;
    leak_[w] = 0.0;
  }
}

double VehicleSim::resonance_hz(std::size_t w) const {
  return tire_[w].f0_hz * std::pow(pressure_fraction(w), tire_[w].freq_exponent);
}

float VehicleSim::sample_yaw_rate() {
  return static_cast<float>(v_ * kappa_ + yaw_bias_ + 0.003 * rng_.gauss());
}

Scenario VehicleSim::active_profile() const {
  if (scenario_ != Scenario::Mixed) return scenario_;
  constexpr Scenario order[] = {Scenario::Highway, Scenario::Country, Scenario::City};
  return order[static_cast<std::size_t>(t_ / 240.0) % 3];
}

double VehicleSim::road_gain() const {
  switch (road_) {
    case Road::Smooth: return 0.5;
    case Road::Rough: return 2.0;
    default: return 1.0;
  }
}

void VehicleSim::new_speed_segment() {
  const Profile& p = profile_for(active_profile());
  if (rng_.uniform() < p.stop_prob) {
    v_target_ = 0.0;
    seg_left_ = rng_.uniform(5.0, 20.0);
  } else {
    v_target_ = rng_.uniform(p.v_lo, p.v_hi) / 3.6;
    seg_left_ = rng_.uniform(p.seg_lo, p.seg_hi);
  }
}

void VehicleSim::new_curve_segment() {
  const Profile& p = profile_for(active_profile());
  if (rng_.uniform() < p.curve_prob) {
    const double radius = rng_.uniform(p.r_lo, p.r_hi);
    kappa_target_ = (rng_.uniform() < 0.5 ? 1.0 : -1.0) / radius;
    curve_left_ = rng_.uniform(p.curve_lo, p.curve_hi);
  } else {
    kappa_target_ = 0.0;
    curve_left_ = rng_.uniform(2.0 * p.curve_lo, 3.0 * p.curve_hi);
  }
}

void VehicleSim::update_driver(double dt) {
  seg_left_ -= dt;
  if (seg_left_ <= 0.0) new_speed_segment();
  curve_left_ -= dt;
  if (curve_left_ <= 0.0) new_curve_segment();

  kappa_ += (kappa_target_ - kappa_) * dt / profile_for(active_profile()).kappa_tau;

  // Slow down for curves (comfortable lateral acceleration), then track target.
  double vt = v_target_;
  const double k = std::max(std::fabs(kappa_target_), std::fabs(kappa_));
  if (k > 1e-9) vt = std::min(vt, std::sqrt(kMaxLatAccel / k));
  const double a_cmd = std::clamp(0.35 * (vt - v_), -2.5, 1.5);
  accel_ += (a_cmd - accel_) * dt / 0.8;
  v_ += accel_ * dt;
  if (v_ < 0.0) {
    v_ = 0.0;
    accel_ = std::max(accel_, 0.0);
  }
}

void VehicleSim::step() {
  const double dt = kStep;
  n_edges_ = 0;
  update_driver(dt);

  for (std::size_t w = 0; w < kWheels; ++w) {
    pressure_[w] = std::max(4.0, pressure_[w] - leak_[w] * dt / 60.0);
  }

  // Longitudinal force on the tires -> slip. Front-wheel drive; braking is
  // split 70/30 front/rear.
  const bool moving = v_ > 0.5;
  const double force = kMass * accel_ + (moving ? 0.5 * kRho * kCdA * v_ * v_ + kCrr * kMass * kG : 0.0);
  const double fz_front = kMass * kG * kFrontWeight;
  const double fz_rear = kMass * kG * (1.0 - kFrontWeight);
  double slip_front = 0.0;
  double slip_rear = 0.0;
  if (force >= 0.0) {
    slip_front = force / (kSlipStiffness * fz_front);
  } else {
    slip_front = 0.7 * force / (kSlipStiffness * fz_front);
    slip_rear = 0.3 * force / (kSlipStiffness * fz_rear);
  }

  // Cornering kinematics: v is the rear-axle centre speed; the front axle runs
  // on a slightly larger arc, the inner wheels on a smaller one.
  const double h = 0.5 * kappa_ * kTrack;
  const double front = std::sqrt(1.0 + kWheelbase * kWheelbase * kappa_ * kappa_);
  const std::array<double, kWheels> lin = {v_ * front * (1.0 - h), v_ * front * (1.0 + h), v_ * (1.0 - h),
                                           v_ * (1.0 + h)};

  const double gain = road_gain();
  const double sqrt_dt = std::sqrt(dt);
  constexpr double kEpsTau = 0.3;
  const double eps_sigma = 3e-4 * gain;
  const double tors_rms = 0.224 * gain * std::sqrt(std::max(v_, 0.0) / 27.8);

  for (std::size_t w = 0; w < kWheels; ++w) {
    const TireParams& tp = tire_[w];
    WheelState& ws = wheel_[w];
    const double pf = pressure_fraction(w);

    ws.road_eps += -ws.road_eps * dt / kEpsTau + eps_sigma * std::sqrt(2.0 * dt / kEpsTau) * rng_.gauss();
    const double radius = kRadius * (1.0 + tp.radius_offset - tp.radius_sensitivity * (1.0 - pf));
    const double slip = w < 2 ? slip_front : slip_rear;
    const double omega = lin[w] * (1.0 + slip) * (1.0 + ws.road_eps) / radius;

    // Sidewall torsional mode (rim vs. belt), driven by road roughness.
    const double wn = 2.0 * kPi * tp.f0_hz * std::pow(pf, tp.freq_exponent);
    const double drive = tors_rms * std::sqrt(4.0 * tp.zeta * wn);
    ws.tors_v += (-2.0 * tp.zeta * wn * ws.tors_v - wn * wn * ws.tors_x) * dt + drive * sqrt_dt * rng_.gauss();
    ws.tors_x += ws.tors_v * dt;

    const double rim = moving ? std::max(0.0, omega + ws.tors_v) : 0.0;
    double prev = ws.phi;
    ws.phi += rim * dt;
    while (ws.phi >= ws.next_edge) {
      const double frac = std::clamp((ws.next_edge - prev) / (ws.phi - prev), 0.0, 1.0);
      const double te = t_ + frac * dt;
      const auto ticks = static_cast<std::uint64_t>(std::llround(te * kTimerHz)) + tick_offset_;
      edges_[n_edges_++] = ToothEdge{static_cast<std::uint32_t>(ticks), static_cast<std::uint8_t>(w)};
      if (++ws.tooth == kTeeth) {
        ws.tooth = 0;
        ws.phi -= 2.0 * kPi;
        prev -= 2.0 * kPi;
      }
      ws.next_edge = (ws.tooth + ws.tooth_err[ws.tooth]) * kPitch;
      if (n_edges_ == edges_.size()) break;
    }
  }
  t_ += dt;

  // A step can hold an edge from each wheel; deliver them in timestamp order.
  std::sort(edges_.begin(), edges_.begin() + static_cast<std::ptrdiff_t>(n_edges_),
            [](const ToothEdge& a, const ToothEdge& b) { return static_cast<std::int32_t>(a.ticks - b.ticks) < 0; });
}

}  // namespace itpms::sim
