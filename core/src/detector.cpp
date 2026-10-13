#include "itpms/detector.hpp"

#include <algorithm>
#include <cmath>

namespace itpms {
namespace {

constexpr float kRadPerTooth = static_cast<float>(2.0 * ct::kPi / kTeeth);
constexpr float kTimerHzF = static_cast<float>(kTimerHz);
constexpr std::uint32_t kStallTicks = kTimerHz / 20;  // 50 ms tooth gap ~ below 3 km/h
constexpr float kToothAlpha = 1.0f / 64.0f;
constexpr std::uint32_t kToothLearnMin = kTeeth * 64;  // ~64 revolutions
constexpr std::uint16_t kMinWindowTeeth = 8;
constexpr std::uint32_t kMinFramesForEstimate = 16;
constexpr float kRefSpeed = 30.0f;  // m/s, feature normalisation

// Ridge prior of the baseline regression, in units of (indicator noise)^2:
// intercept almost free, speed / accel slopes mildly regularised so a
// calibration drive at near-constant speed stays well conditioned.
constexpr std::array<float, 3> kPrior = {1.0e4f, 2.5e3f, 2.5e3f};

constexpr auto kHann = ct::make_hann<kFftSize>();

// Quadratic fit of log-power over +-kFitHalf bins around the peak.
constexpr int kFitHalf = 5;
constexpr float fit_sum_x2() {
  float s = 0.0f;
  for (int x = -kFitHalf; x <= kFitHalf; ++x) s += static_cast<float>(x * x);
  return s;
}
constexpr float kFitSx2 = fit_sum_x2();
constexpr float kFitMeanX2 = kFitSx2 / static_cast<float>(2 * kFitHalf + 1);
constexpr float fit_sum_q2() {
  float s = 0.0f;
  for (int x = -kFitHalf; x <= kFitHalf; ++x) {
    const float q = static_cast<float>(x * x) - kFitMeanX2;
    s += q * q;
  }
  return s;
}
constexpr float kFitSq2 = fit_sum_q2();

constexpr float kDetrendSc2 = [] {
  float s = 0.0f;
  for (std::size_t i = 0; i < kFftSize; ++i) {
    const float c = static_cast<float>(i) - 0.5f * static_cast<float>(kFftSize - 1);
    s += c * c;
  }
  return s;
}();

// Wrap-safe "now >= deadline" for a free-running 32-bit timer.
inline bool reached(std::uint32_t now, std::uint32_t deadline) noexcept {
  return static_cast<std::int32_t>(now - deadline) >= 0;
}

// Median of a small fixed window: copy, then partial sort (no heap).
float median(const std::array<float, kMedianTaps>& v, std::size_t n) noexcept {
  std::array<float, kMedianTaps> tmp = v;
  const auto mid = tmp.begin() + static_cast<std::ptrdiff_t>(n / 2);
  std::nth_element(tmp.begin(), mid, tmp.begin() + static_cast<std::ptrdiff_t>(n));
  return *mid;
}

float estimate_peak_hz(const std::array<float, kPsdBins>& psd) noexcept {
  std::size_t best = kFitHalf;
  for (std::size_t k = kFitHalf; k < kPsdBins - kFitHalf; ++k) {
    if (psd[k] > psd[best]) best = k;
  }
  float s1 = 0.0f;
  float s2 = 0.0f;
  for (int x = -kFitHalf; x <= kFitHalf; ++x) {
    const auto idx = static_cast<std::size_t>(static_cast<int>(best) + x);
    const float y = std::log(std::max(psd[idx], 1.0e-12f));
    s1 += static_cast<float>(x) * y;
    s2 += (static_cast<float>(x * x) - kFitMeanX2) * y;
  }
  const float a1 = s1 / kFitSx2;
  const float a2 = s2 / kFitSq2;
  float offset = 0.0f;
  if (a2 < 0.0f) {
    offset = std::clamp(-a1 / (2.0f * a2), -static_cast<float>(kFitHalf), static_cast<float>(kFitHalf));
  }
  return (static_cast<float>(kPsdLoBin + best) + offset) * kBinHz;
}

}  // namespace

Detector::Detector(const DetectorConfig& cfg) noexcept
    : cfg_(cfg),
      radius_period_ticks_(static_cast<std::uint32_t>(cfg.radius_period_s * kTimerHzF)) {
  for (auto& tt : tooth_) tt.corr.fill(1.0f);
  reset_learning();
}

void Detector::reset_learning() noexcept {
  model_ = RadiusModel{};
  for (std::size_t i = 0; i < 3; ++i) model_.P[i][i] = kPrior[i];
  for (auto& s : spec_) s = Spectrum{};
  status_.state = SystemState::Calibrating;
  status_.calibration_progress = 0.0f;
  status_.all_low = false;
  status_.wheel = {};
  status_.residual = {};
  status_.noise = {};
  status_.radius_samples = 0;
  status_.freq_frames = {};
  status_.gates = {};
}

void Detector::request_recalibration() noexcept { reset_learning(); }

void Detector::process() noexcept {
  Edge e{};
  while (queue_.pop(e)) handle_edge(e);
}

Status Detector::status() const noexcept {
  Status s = status_;
  s.edges_dropped = dropped_.load(std::memory_order_relaxed);
  return s;
}

void Detector::restart_motion(ToothTracker& tt, Resampler& rs, std::uint32_t t) noexcept {
  tt.period.fill(0);
  tt.rev_sum = 0;
  tt.filled = 0;
  tt.win_start = t;
  tt.win_end = t;
  tt.win_teeth = 0;
  rs.active = false;
  rs.fill = 0;
  rs.count = 0;
}

void Detector::handle_edge(const Edge& e) noexcept {
  const std::size_t w = e.wheel;
  ToothTracker& tt = tooth_[w];
  Resampler& rs = rs_[w];
  ++status_.edges_processed;

  if (!clock_started_) {
    clock_started_ = true;
    next_radius_eval_ = e.t + radius_period_ticks_;
  }
  if (!tt.has_last) {
    tt.has_last = true;
    tt.last_edge = e.t;
    restart_motion(tt, rs, e.t);
    return;
  }

  const std::uint32_t period = e.t - tt.last_edge;
  if (period == 0) return;  // duplicate capture
  tt.last_edge = e.t;

  // The tone ring keeps turning while we are stalled, so the tooth index stays
  // aligned; only the timing history is discarded.
  const std::uint8_t k = tt.tooth;
  tt.tooth = static_cast<std::uint8_t>(k + 1 == kTeeth ? 0 : k + 1);
  if (period > kStallTicks) {
    restart_motion(tt, rs, e.t);
    return;
  }

  // --- Tone-ring error learning ------------------------------------------
  tt.rev_sum += period - tt.period[k];  // modular arithmetic keeps the sum exact
  tt.period[k] = period;
  if (tt.filled < kTeeth) {
    ++tt.filled;
  } else {
    const float ratio = static_cast<float>(period) * static_cast<float>(kTeeth) / static_cast<float>(tt.rev_sum);
    tt.corr[k] += kToothAlpha * (ratio - tt.corr[k]);
    if (tt.learned < kToothLearnMin) ++tt.learned;
    if (k == kTeeth - 1) {
      // Acceleration biases every tooth equally; keep the mean correction at 1.
      float sum = 0.0f;
      for (float c : tt.corr) sum += c;
      const float inv = static_cast<float>(kTeeth) / sum;
      for (float& c : tt.corr) c *= inv;
    }
  }
  const float speed = kRadPerTooth * kTimerHzF * tt.corr[k] / static_cast<float>(period);  // rad/s

  // --- Uniform resampling (linear interpolation) ---------------------------
  if (!rs.active) {
    rs.active = true;
    rs.next_t = e.t + kResampleTicks;
  } else {
    const float span = static_cast<float>(e.t - rs.prev_t);
    while (reached(e.t, rs.next_t)) {
      const float f = static_cast<float>(rs.next_t - rs.prev_t) / span;
      push_sample(w, rs.prev_speed + f * (speed - rs.prev_speed));
      rs.next_t += kResampleTicks;
    }
  }
  rs.prev_t = e.t;
  rs.prev_speed = speed;

  // --- Rolling-radius window -----------------------------------------------
  tt.win_end = e.t;
  if (tt.win_teeth < UINT16_MAX) ++tt.win_teeth;
  if (reached(e.t, next_radius_eval_)) {
    evaluate_radius();
    next_radius_eval_ += radius_period_ticks_;
    if (reached(e.t, next_radius_eval_)) next_radius_eval_ = e.t + radius_period_ticks_;
  }
}

void Detector::push_sample(std::size_t w, float speed) noexcept {
  Resampler& rs = rs_[w];
  rs.ring[rs.pos] = speed;
  rs.pos = static_cast<std::uint16_t>((rs.pos + 1u) & (kFftSize - 1u));
  if (rs.fill < kFftSize) ++rs.fill;
  ++rs.count;
  // Stagger the four wheels by a quarter hop so FFTs never pile up in one call.
  const std::size_t phase = (rs.count + w * (kFftHop / kWheels)) % kFftHop;
  if (rs.fill == kFftSize && phase == 0) run_frame(w);
}

void Detector::run_frame(std::size_t w) noexcept {
  const bool usable = status_.speed_kmh >= cfg_.freq_min_speed_kmh && status_.speed_kmh <= cfg_.max_speed_kmh &&
                      !vehicle_.brake && std::fabs(status_.accel_mps2) <= cfg_.freq_max_accel &&
                      tooth_[w].learned >= kToothLearnMin;
  if (!usable) return;

  // Unroll the ring chronologically, remove mean + linear trend, apply window.
  const Resampler& rs = rs_[w];
  float mean = 0.0f;
  float slope = 0.0f;
  for (std::size_t i = 0; i < kFftSize; ++i) {
    const float v = rs.ring[(rs.pos + i) & (kFftSize - 1)];
    frame_[i] = v;
    mean += v;
    slope += (static_cast<float>(i) - 0.5f * static_cast<float>(kFftSize - 1)) * v;
  }
  mean /= static_cast<float>(kFftSize);
  slope /= kDetrendSc2;
  float sq = 0.0f;
  float peak = 0.0f;
  for (std::size_t i = 0; i < kFftSize; ++i) {
    const float c = static_cast<float>(i) - 0.5f * static_cast<float>(kFftSize - 1);
    frame_[i] -= mean + slope * c;
    sq += frame_[i] * frame_[i];
    peak = std::max(peak, std::fabs(frame_[i]));
  }
  // Road texture gives a Gaussian-like signal (crest factor ~3-4). A stud
  // strike or pothole is a sharp spike that would whiten the averaged
  // spectrum. Clip (winsorise) instead of dropping the frame, so a road that
  // is studded for minutes never starves the estimator. Two passes: the
  // first limit is inflated by the spike itself.
  if (cfg_.impulse_clip_sigma > 0.0f) {
    float limit = cfg_.impulse_clip_sigma * std::sqrt(sq / static_cast<float>(kFftSize));
    if (peak > limit) {
      ++status_.impulse_frames;
      for (int pass = 0; pass < 2; ++pass) {
        sq = 0.0f;
        for (float& v : frame_) {
          v = std::clamp(v, -limit, limit);
          sq += v * v;
        }
        limit = cfg_.impulse_clip_sigma * std::sqrt(sq / static_cast<float>(kFftSize));
      }
    }
  }
  for (std::size_t i = 0; i < kFftSize; ++i) frame_[i] *= kHann[i];

  RealFft<kFftSize>::forward(frame_.data(), bins_.data());

  // Band power, normalised so fast and slow frames carry equal weight.
  std::array<float, kPsdBins> p{};
  float total = 0.0f;
  for (std::size_t k = 0; k < kPsdBins; ++k) {
    const Complex& b = bins_[kPsdLoBin + k];
    p[k] = b.re * b.re + b.im * b.im;
    total += p[k];
  }
  if (!(total > 0.0f)) return;

  Spectrum& s = spec_[w];
  const float alpha = std::max(cfg_.psd_alpha, 1.0f / static_cast<float>(s.frames + 1));
  const float inv_total = 1.0f / total;
  for (std::size_t k = 0; k < kPsdBins; ++k) s.psd[k] += alpha * (p[k] * inv_total - s.psd[k]);
  if (s.frames < UINT32_MAX) ++s.frames;
  if (s.frames >= kMinFramesForEstimate) s.f_est = estimate_peak_hz(s.psd);
  status_.freq_frames[w] = s.frames;

  maybe_finish_calibration();
  update_estimates();
}

void Detector::evaluate_radius() noexcept {
  std::array<float, kWheels> rate{};
  bool valid = true;
  for (std::size_t w = 0; w < kWheels; ++w) {
    ToothTracker& tt = tooth_[w];
    if (tt.win_teeth < kMinWindowTeeth || tt.win_end == tt.win_start) valid = false;
    else rate[w] = static_cast<float>(tt.win_teeth) / static_cast<float>(tt.win_end - tt.win_start);
    tt.win_start = tt.win_end;
    tt.win_teeth = 0;
  }
  if (!valid) {
    status_.speed_kmh = 0.0f;
    prev_speed_mps_ = -1.0f;
    ++status_.gates[static_cast<std::size_t>(Gate::Speed)];
    return;
  }

  const float mean = 0.25f * (rate[FL] + rate[FR] + rate[RL] + rate[RR]);
  const float v = mean * kTimerHzF * kRadPerTooth * cfg_.rolling_radius_m;
  if (prev_speed_mps_ >= 0.0f) {
    const float raw = (v - prev_speed_mps_) / cfg_.radius_period_s;
    status_.accel_mps2 += 0.5f * (raw - status_.accel_mps2);
  }
  prev_speed_mps_ = v;
  status_.speed_kmh = v * 3.6f;

  float spread = 0.0f;
  for (float r : rate) spread = std::max(spread, std::fabs(r / mean - 1.0f));

  Gate gate = Gate::Accepted;
  if (status_.speed_kmh < cfg_.min_speed_kmh || status_.speed_kmh > cfg_.max_speed_kmh) gate = Gate::Speed;
  else if (vehicle_.brake) gate = Gate::Brake;
  else if (std::fabs(status_.accel_mps2) > cfg_.max_long_accel) gate = Gate::Accel;
  else if (std::fabs(v * vehicle_.yaw_rate_rps) > cfg_.max_lat_accel) gate = Gate::Cornering;
  else if (spread > cfg_.max_wheel_spread) gate = Gate::WheelSpread;
  ++status_.gates[static_cast<std::size_t>(gate)];
  if (gate != Gate::Accepted) return;

  std::array<float, kIndicators> x{};
  x[kDiagonal] = (rate[FL] + rate[RR]) / (rate[FR] + rate[RL]) - 1.0f;
  x[kAxle] = (rate[FL] + rate[FR]) / (rate[RL] + rate[RR]) - 1.0f;
  // In a left turn the left wheels follow a shorter arc: remove the kinematic
  // left/right speed ratio (1 - kT/2) / (1 + kT/2) using the yaw sensor.
  const float h = 0.5f * cfg_.track_width_m * vehicle_.yaw_rate_rps / v;
  x[kSide] = (rate[FL] + rate[RL]) / (rate[FR] + rate[RR]) * (1.0f + h) / (1.0f - h) - 1.0f;

  // ~2 s running median: robust to isolated impulsive samples.
  auto& m = model_;
  for (std::size_t j = 0; j < kIndicators; ++j) m.recent[j][m.recent_pos] = x[j];
  m.recent_pos = static_cast<std::uint8_t>((m.recent_pos + 1u) % kMedianTaps);
  if (m.recent_fill < kMedianTaps) ++m.recent_fill;
  if (cfg_.median_prefilter)
    for (std::size_t j = 0; j < kIndicators; ++j) x[j] = median(m.recent[j], m.recent_fill);

  const float vn = v / kRefSpeed;
  const std::array<float, 3> phi = {1.0f, vn * vn, status_.accel_mps2};

  if (status_.state == SystemState::Calibrating) {
    rls_update(phi, x);
  } else {
    for (std::size_t j = 0; j < kIndicators; ++j) {
      const auto& th = model_.theta[j];
      const float e = x[j] - (th[0] * phi[0] + th[1] * phi[1] + th[2] * phi[2]);
      model_.ewma[j] += cfg_.radius_ewma_alpha * (e - model_.ewma[j]);
    }
  }
  if (model_.n < UINT32_MAX) ++model_.n;
  status_.radius_samples = model_.n;

  maybe_finish_calibration();
  update_estimates();
}

void Detector::rls_update(const std::array<float, 3>& phi, const std::array<float, kIndicators>& x) noexcept {
  auto& P = model_.P;
  std::array<float, 3> pphi{};
  for (std::size_t i = 0; i < 3; ++i) pphi[i] = P[i][0] * phi[0] + P[i][1] * phi[1] + P[i][2] * phi[2];
  const float denom = 1.0f + phi[0] * pphi[0] + phi[1] * pphi[1] + phi[2] * pphi[2];
  std::array<float, 3> gain{};
  for (std::size_t i = 0; i < 3; ++i) gain[i] = pphi[i] / denom;

  // Variance of the a-priori error after a short warm-up (slightly pessimistic,
  // which only makes the reported confidence conservative).
  constexpr std::uint32_t kWarmup = 50;
  for (std::size_t j = 0; j < kIndicators; ++j) {
    auto& th = model_.theta[j];
    const float e = x[j] - (th[0] * phi[0] + th[1] * phi[1] + th[2] * phi[2]);
    for (std::size_t i = 0; i < 3; ++i) th[i] += gain[i] * e;
    model_.ewma[j] += cfg_.radius_ewma_alpha * (e - model_.ewma[j]);
    if (model_.n >= kWarmup) {
      const float a = std::max(1.0f / static_cast<float>(model_.n - kWarmup + 1), 1.0f / 1000.0f);
      model_.err_var[j] += a * (e * e - model_.err_var[j]);
    }
  }

  for (std::size_t i = 0; i < 3; ++i)
    for (std::size_t j = 0; j < 3; ++j) P[i][j] -= gain[i] * pphi[j];
  // Re-symmetrise: float RLS slowly loses symmetry otherwise.
  for (std::size_t i = 0; i < 3; ++i)
    for (std::size_t j = i + 1; j < 3; ++j) P[i][j] = P[j][i] = 0.5f * (P[i][j] + P[j][i]);
}

void Detector::maybe_finish_calibration() noexcept {
  if (status_.state != SystemState::Calibrating) return;

  float progress = static_cast<float>(model_.n) / static_cast<float>(cfg_.cal_radius_samples);
  for (const Spectrum& s : spec_) {
    progress = std::min(progress, static_cast<float>(s.frames) / static_cast<float>(cfg_.cal_freq_frames));
  }
  status_.calibration_progress = std::min(progress, 1.0f);
  if (progress < 1.0f) return;

  for (Spectrum& s : spec_) {
    s.f_ref = s.f_est;
    s.psd_ref = s.psd;
  }
  model_.ewma = {};
  model_.n = 0;
  status_.state = SystemState::Monitoring;
}

void Detector::update_estimates() noexcept {
  for (std::size_t w = 0; w < kWheels; ++w) {
    status_.wheel[w].freq_hz = spec_[w].f_est;
    status_.wheel[w].freq_ref_hz = spec_[w].f_ref;
  }
  const float ewma_gain = std::sqrt(cfg_.radius_ewma_alpha / (2.0f - cfg_.radius_ewma_alpha));
  for (std::size_t j = 0; j < kIndicators; ++j) {
    status_.residual[j] = model_.ewma[j];
    status_.noise[j] = std::sqrt(model_.err_var[j]) * ewma_gain;
  }
  if (status_.state != SystemState::Monitoring) return;

  // Indicators are orthogonal +-1 patterns over the wheels; invert them to get
  // each wheel's speed excess relative to the four-wheel mean.
  const float d = model_.ewma[kDiagonal];
  const float s = model_.ewma[kSide];
  const float a = model_.ewma[kAxle];
  const std::array<float, kWheels> excess = {0.5f * (d + s + a), 0.5f * (-d - s + a), 0.5f * (-d + s - a),
                                             0.5f * (d - s - a)};

  // Resonance shift -> absolute loss: p/p_ref = (f/f_ref)^(1/exponent).
  float common = 0.0f;
  for (std::size_t w = 0; w < kWheels; ++w) {
    const Spectrum& sp = spec_[w];
    const float ratio = sp.f_ref > 0.0f ? sp.f_est / sp.f_ref : 1.0f;
    status_.wheel[w].loss_freq = 1.0f - std::pow(ratio, 1.0f / cfg_.freq_exponent);
    common += status_.wheel[w].loss_freq;
  }
  common *= 0.25f;

  // Fusion: radius gives precise relative differences, resonance supplies the
  // common mode (all four tires low) that radius analysis is blind to.
  bool all = true;
  for (std::size_t w = 0; w < kWheels; ++w) {
    WheelStatus& ws = status_.wheel[w];
    ws.loss_radius = excess[w] / cfg_.radius_sensitivity;
    ws.loss = ws.loss_radius + common;
    if (ws.loss >= cfg_.warn_loss) ws.warning = true;
    else if (ws.loss < cfg_.clear_loss) ws.warning = false;
    all = all && ws.warning;
  }
  status_.all_low = all;
}

MemoryReport Detector::memory_report() noexcept {
  MemoryReport r{};
  r.total = sizeof(Detector);
  r.edge_queue = sizeof(SpscRing<Edge, kEdgeQueueSize>);
  r.tooth_tracking = sizeof(std::array<ToothTracker, kWheels>);
  r.resampler = sizeof(std::array<Resampler, kWheels>);
  r.spectra = sizeof(std::array<Spectrum, kWheels>);
  r.fft_scratch = sizeof(std::array<float, kFftSize>) + sizeof(std::array<Complex, RealFft<kFftSize>::kBins>);
  r.radius_model = sizeof(RadiusModel);
  r.budget = kRamBudgetBytes;
  return r;
}

}  // namespace itpms
