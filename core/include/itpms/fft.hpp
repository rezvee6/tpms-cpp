// Real-input radix-2 FFT, N real samples via one N/2-point complex FFT plus
// an in-place split pass. All tables (twiddles, bit reversal) are constexpr,
// so they live in flash; the only RAM used is the caller's N/2+1 buffer.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "itpms/ct_math.hpp"

namespace itpms {

struct Complex {
  float re;
  float im;
};

template <std::size_t N>
class RealFft {
  static_assert(ct::is_pow2(N) && N >= 8, "RealFft size must be a power of two >= 8");

 public:
  static constexpr std::size_t kHalf = N / 2;
  static constexpr std::size_t kBins = kHalf + 1;  // DC .. Nyquist

  // Transforms `in[N]` into `buf[kBins]` (unnormalised DFT bins 0..N/2).
  static void forward(const float* in, Complex* buf) noexcept {
    // Pack even/odd samples as one complex sequence, scattered in bit-reversed order.
    for (std::size_t i = 0; i < kHalf; ++i) {
      buf[kBitrev[i]] = Complex{in[2 * i], in[2 * i + 1]};
    }
    // Iterative Cooley-Tukey on kHalf points. W_len^j == W_N^(j*N/len).
    for (std::size_t len = 2; len <= kHalf; len <<= 1) {
      const std::size_t half = len / 2;
      const std::size_t stride = N / len;
      for (std::size_t base = 0; base < kHalf; base += len) {
        for (std::size_t j = 0; j < half; ++j) {
          const Complex w = kTwiddle[j * stride];
          Complex& a = buf[base + j];
          Complex& b = buf[base + j + half];
          const Complex t{w.re * b.re - w.im * b.im, w.re * b.im + w.im * b.re};
          b = Complex{a.re - t.re, a.im - t.im};
          a = Complex{a.re + t.re, a.im + t.im};
        }
      }
    }
    // Split Z[k] into X[k] and X[M-k] pairwise so it can run in place.
    const Complex z0 = buf[0];
    buf[0] = Complex{z0.re + z0.im, 0.0f};
    buf[kHalf] = Complex{z0.re - z0.im, 0.0f};
    for (std::size_t k = 1; k <= kHalf / 2; ++k) {
      const Complex a = buf[k];
      const Complex b = buf[kHalf - k];
      buf[k] = split(a, b, kTwiddle[k]);
      if (k != kHalf - k) {
        buf[kHalf - k] = split(b, a, kTwiddle[kHalf - k]);
      }
    }
  }

 private:
  // X[k] = E + W^k * O, E = (Z[k] + conj Z[M-k]) / 2, O = (Z[k] - conj Z[M-k]) / 2j
  static Complex split(Complex zk, Complex zmk, Complex w) noexcept {
    const float er = 0.5f * (zk.re + zmk.re);
    const float ei = 0.5f * (zk.im - zmk.im);
    const float or_ = 0.5f * (zk.im + zmk.im);
    const float oi = -0.5f * (zk.re - zmk.re);
    return Complex{er + w.re * or_ - w.im * oi, ei + w.re * oi + w.im * or_};
  }

  static constexpr std::array<Complex, kHalf> make_twiddles() {
    std::array<Complex, kHalf> t{};
    for (std::size_t k = 0; k < kHalf; ++k) {
      const double ang = -2.0 * ct::kPi * static_cast<double>(k) / static_cast<double>(N);
      t[k] = Complex{static_cast<float>(ct::cos(ang)), static_cast<float>(ct::sin(ang))};
    }
    return t;
  }

  static constexpr std::array<std::uint16_t, kHalf> make_bitrev() {
    std::array<std::uint16_t, kHalf> r{};
    constexpr std::size_t bits = ct::log2(kHalf);
    for (std::size_t i = 0; i < kHalf; ++i) {
      std::size_t v = 0;
      for (std::size_t b = 0; b < bits; ++b) {
        v |= ((i >> b) & 1u) << (bits - 1 - b);
      }
      r[i] = static_cast<std::uint16_t>(v);
    }
    return r;
  }

  static constexpr std::array<Complex, kHalf> kTwiddle = make_twiddles();
  static constexpr std::array<std::uint16_t, kHalf> kBitrev = make_bitrev();
};

}  // namespace itpms
