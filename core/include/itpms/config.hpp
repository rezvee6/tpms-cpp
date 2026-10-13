// Compile-time constants (they size every buffer) and runtime tunables for the
// indirect TPMS detector. Everything here targets a Cortex-M4F-class ECU:
// single-precision float only, no heap, fixed RAM budget.
#pragma once

#include <cstddef>
#include <cstdint>

namespace itpms {

enum Wheel : std::uint8_t { FL = 0, FR = 1, RL = 2, RR = 3 };
inline constexpr std::size_t kWheels = 4;

// --- Hardware -------------------------------------------------------------
inline constexpr std::uint32_t kTimerHz = 10'000'000;  // input-capture timer, wraps every ~429 s
inline constexpr std::uint32_t kTeeth = 48;            // ABS tone-ring teeth per revolution

// --- Frequency analysis (torsional resonance) ------------------------------
inline constexpr std::uint32_t kResampleHz = 250;  // uniform wheel-speed sample rate
inline constexpr std::uint32_t kResampleTicks = kTimerHz / kResampleHz;
static_assert(kTimerHz % kResampleHz == 0, "resample grid must be an integer number of ticks");

inline constexpr std::size_t kFftSize = 256;  // ~1 s frames, 0.98 Hz bins
inline constexpr std::size_t kFftHop = kFftSize / 2;
inline constexpr std::size_t kPsdLoBin = 16;  // 15.6 Hz
inline constexpr std::size_t kPsdBins = 64;   // .. 77.1 Hz
inline constexpr float kBinHz = static_cast<float>(kResampleHz) / static_cast<float>(kFftSize);
static_assert(kPsdLoBin + kPsdBins <= kFftSize / 2, "PSD band must lie below Nyquist");

// --- Plumbing --------------------------------------------------------------
inline constexpr std::size_t kMedianTaps = 9;  // ~2.25 s pre-filter on rolling-radius samples
inline constexpr std::size_t kEdgeQueueSize = 256;  // ISR -> task, ~35 ms of edges at 250 km/h
inline constexpr std::size_t kRamBudgetBytes = 32 * 1024;

struct DetectorConfig {
  // Vehicle / tire model (nominal values for the fitted tire size).
  float rolling_radius_m = 0.31f;
  float track_width_m = 1.60f;
  float radius_sensitivity = 0.010f;  // fractional wheel-speed rise per fractional pressure loss
  float freq_exponent = 0.25f;        // resonance f ~ pressure^exponent

  // Rolling-radius sample gating.
  float min_speed_kmh = 25.0f;
  float max_speed_kmh = 200.0f;
  float max_long_accel = 0.8f;   // m/s^2, derived from wheel speeds
  float max_lat_accel = 2.0f;    // m/s^2, speed * yaw rate
  float max_wheel_spread = 0.02f;  // any wheel this far from the mean => slip / ABS event

  // Frequency-analysis gating.
  float freq_min_speed_kmh = 40.0f;
  float freq_max_accel = 2.0f;

  // Impulse robustness (cat's eyes, potholes, expansion joints).
  bool median_prefilter = true;   // ~2 s running median on rolling-radius samples
  float impulse_clip_sigma = 4.5f;  // clip resampled speed beyond this many RMS (0 = off)

  // Time constants.
  float radius_period_s = 0.25f;      // one rolling-radius sample per period
  float radius_ewma_alpha = 1.0f / 400.0f;  // ~100 s of accepted samples
  float psd_alpha = 1.0f / 64.0f;     // ~32 s of accepted frames

  // Calibration ("TPMS reset" button) requirements.
  std::uint32_t cal_radius_samples = 1200;  // ~5 min of accepted driving
  std::uint32_t cal_freq_frames = 240;      // ~2 min per wheel above 40 km/h

  // Warning hysteresis on estimated pressure loss.
  float warn_loss = 0.20f;
  float clear_loss = 0.12f;
};

}  // namespace itpms
