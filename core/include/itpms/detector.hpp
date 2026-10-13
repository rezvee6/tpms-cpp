// Indirect TPMS detector: consumes raw ABS tooth-edge timestamps and estimates
// per-wheel pressure loss with two complementary methods.
//
//  * Rolling radius: an under-inflated tire has a smaller rolling radius and
//    spins faster. Diagonal / side / axle wheel-speed ratios are compared with a
//    baseline learned at calibration. Sees relative loss only (not all four).
//  * Torsional resonance: the sidewall is a spring; its rim-speed resonance
//    (~40 Hz) drops with pressure. Gives an absolute, per-wheel estimate, which
//    supplies the common mode the radius method cannot see.
//
// Embedded contract:
//  * on_tooth_edge() is ISR-safe, O(1), wait-free.
//  * process() runs in a background task; bounded work per edge plus at most
//    one 256-point FFT per wheel per 0.5 s, staggered across wheels.
//  * No heap, no exceptions, no RTTI, float only. sizeof(Detector) is
//    statically checked against the RAM budget.
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "itpms/config.hpp"
#include "itpms/fft.hpp"
#include "itpms/spsc_ring.hpp"

namespace itpms {

// Low-rate CAN signals (ESC yaw sensor, brake switch), ~100 Hz.
struct VehicleSignals {
  float yaw_rate_rps = 0.0f;  // positive = turning left
  bool brake = false;
};

enum class SystemState : std::uint8_t { Calibrating, Monitoring };

// Why a rolling-radius sample was (not) used.
enum class Gate : std::uint8_t { Accepted, Speed, Brake, Accel, Cornering, WheelSpread, Count };
inline constexpr std::size_t kGateCount = static_cast<std::size_t>(Gate::Count);

// Rolling-radius indicators (wheel-speed ratios minus one).
enum Indicator : std::uint8_t { kDiagonal = 0, kSide = 1, kAxle = 2 };
inline constexpr std::size_t kIndicators = 3;

struct WheelStatus {
  float loss = 0.0f;         // fused pressure-loss estimate (0.25 = 25 % under-inflated)
  float loss_radius = 0.0f;  // rolling-radius part, relative to the mean of all four
  float loss_freq = 0.0f;    // resonance-shift estimate, absolute
  float freq_hz = 0.0f;      // current torsional resonance estimate
  float freq_ref_hz = 0.0f;  // resonance learned at calibration
  bool warning = false;
};

struct Status {
  SystemState state = SystemState::Calibrating;
  float calibration_progress = 0.0f;  // 0..1
  float speed_kmh = 0.0f;
  float accel_mps2 = 0.0f;
  std::array<WheelStatus, kWheels> wheel{};
  std::array<float, kIndicators> residual{};  // filtered deviation from baseline
  std::array<float, kIndicators> noise{};     // 1-sigma of the filtered residual
  bool all_low = false;
  std::uint32_t radius_samples = 0;
  std::array<std::uint32_t, kWheels> freq_frames{};
  std::uint32_t impulse_frames = 0;  // FFT frames that needed impulse clipping
  std::array<std::uint32_t, kGateCount> gates{};
  std::uint32_t edges_processed = 0;
  std::uint32_t edges_dropped = 0;
};

struct MemoryReport {
  std::size_t total;
  std::size_t edge_queue;
  std::size_t tooth_tracking;
  std::size_t resampler;
  std::size_t spectra;
  std::size_t fft_scratch;
  std::size_t radius_model;
  std::size_t budget;
};

class Detector {
 public:
  explicit Detector(const DetectorConfig& cfg = {}) noexcept;

  // ISR context: record one tooth edge (input-capture timestamp). Returns
  // false if the queue overflowed (the task fell behind).
  bool on_tooth_edge(Wheel w, std::uint32_t timestamp) noexcept {
    if (queue_.push(Edge{timestamp, w})) return true;
    dropped_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }

  // Task context.
  void on_vehicle_signals(const VehicleSignals& s) noexcept { vehicle_ = s; }
  void process() noexcept;
  void request_recalibration() noexcept;  // driver pressed "TPMS reset"

  Status status() const noexcept;
  const std::array<float, kPsdBins>& spectrum(Wheel w) const noexcept { return spec_[w].psd; }
  const std::array<float, kPsdBins>& reference_spectrum(Wheel w) const noexcept { return spec_[w].psd_ref; }
  const DetectorConfig& config() const noexcept { return cfg_; }

  static MemoryReport memory_report() noexcept;

 private:
  struct Edge {
    std::uint32_t t;
    Wheel wheel;
  };

  // Per-wheel tooth timing with tone-ring error learning: each tooth gap is
  // learned relative to the mean gap, so machining errors don't show up as
  // spurious speed ripple at rotation harmonics (which sweep through 20-60 Hz).
  struct ToothTracker {
    std::array<std::uint32_t, kTeeth> period{};
    std::array<float, kTeeth> corr{};
    std::uint32_t last_edge = 0;
    std::uint32_t rev_sum = 0;
    std::uint32_t learned = 0;
    std::uint16_t filled = 0;
    std::uint8_t tooth = 0;
    bool has_last = false;
    // Rolling-radius window: whole teeth between two edge timestamps.
    std::uint32_t win_start = 0;
    std::uint32_t win_end = 0;
    std::uint16_t win_teeth = 0;
  };

  // Converts irregular per-tooth speed samples to a uniform 250 Hz stream.
  struct Resampler {
    std::array<float, kFftSize> ring{};
    std::uint32_t prev_t = 0;
    std::uint32_t next_t = 0;
    std::uint32_t count = 0;
    float prev_speed = 0.0f;
    std::uint16_t pos = 0;
    std::uint16_t fill = 0;
    bool active = false;
  };

  struct Spectrum {
    std::array<float, kPsdBins> psd{};
    std::array<float, kPsdBins> psd_ref{};
    std::uint32_t frames = 0;
    float f_est = 0.0f;
    float f_ref = 0.0f;
  };

  // Baseline model for each indicator: x = theta . [1, (v/30)^2, accel],
  // fitted by recursive least squares during calibration (shared covariance:
  // all three indicators use the same regressors), then frozen.
  struct RadiusModel {
    // Running median of the last kMedianTaps accepted samples per indicator:
    // a stud strike or pothole is an outlier for a sample or two and is
    // discarded, while a real pressure change passes with ~1 s of delay.
    std::array<std::array<float, kMedianTaps>, kIndicators> recent{};
    std::uint8_t recent_pos = 0;
    std::uint8_t recent_fill = 0;
    std::array<std::array<float, 3>, 3> P{};
    std::array<std::array<float, 3>, kIndicators> theta{};
    std::array<float, kIndicators> err_var{};
    std::array<float, kIndicators> ewma{};
    std::uint32_t n = 0;
  };

  void handle_edge(const Edge& e) noexcept;
  void restart_motion(ToothTracker& tt, Resampler& rs, std::uint32_t t) noexcept;
  void push_sample(std::size_t w, float speed) noexcept;
  void run_frame(std::size_t w) noexcept;
  void evaluate_radius() noexcept;
  void rls_update(const std::array<float, 3>& phi, const std::array<float, kIndicators>& x) noexcept;
  void maybe_finish_calibration() noexcept;
  void update_estimates() noexcept;
  void reset_learning() noexcept;

  DetectorConfig cfg_;
  SpscRing<Edge, kEdgeQueueSize> queue_;
  std::atomic<std::uint32_t> dropped_{0};
  VehicleSignals vehicle_{};

  std::array<ToothTracker, kWheels> tooth_{};
  std::array<Resampler, kWheels> rs_{};
  std::array<Spectrum, kWheels> spec_{};
  RadiusModel model_{};

  // FFT scratch, shared by all wheels (frames never overlap in time).
  std::array<float, kFftSize> frame_{};
  std::array<Complex, RealFft<kFftSize>::kBins> bins_{};

  std::uint32_t radius_period_ticks_;
  std::uint32_t next_radius_eval_ = 0;
  bool clock_started_ = false;
  float prev_speed_mps_ = -1.0f;

  Status status_{};
};

static_assert(sizeof(Detector) <= kRamBudgetBytes, "detector exceeds the ECU RAM budget");

}  // namespace itpms
