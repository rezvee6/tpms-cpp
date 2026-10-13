// Compile-time trigonometry so window and twiddle tables are baked into
// .rodata (flash) instead of being computed at boot or stored in RAM.
// The double math here only ever runs inside the compiler.
#pragma once

#include <array>
#include <cstddef>

namespace itpms::ct {

inline constexpr double kPi = 3.14159265358979323846;

constexpr double wrap_pi(double x) {
  const double two_pi = 2.0 * kPi;
  const auto k = static_cast<long long>(x / two_pi + (x >= 0.0 ? 0.5 : -0.5));
  return x - static_cast<double>(k) * two_pi;
}

// Taylor series on [-pi, pi]; 13 terms keeps the error below 1e-12.
constexpr double sin(double x) {
  x = wrap_pi(x);
  double term = x;
  double sum = x;
  for (int n = 1; n < 13; ++n) {
    term *= -x * x / static_cast<double>((2 * n) * (2 * n + 1));
    sum += term;
  }
  return sum;
}

constexpr double cos(double x) { return sin(x + kPi / 2.0); }

constexpr bool is_pow2(std::size_t n) { return n >= 2 && (n & (n - 1)) == 0; }

constexpr std::size_t log2(std::size_t n) {
  std::size_t r = 0;
  while (n > 1) {
    n >>= 1;
    ++r;
  }
  return r;
}

// Periodic Hann window (the right choice for overlapped spectral averaging).
template <std::size_t N>
constexpr std::array<float, N> make_hann() {
  std::array<float, N> w{};
  for (std::size_t i = 0; i < N; ++i) {
    w[i] = static_cast<float>(0.5 - 0.5 * cos(2.0 * kPi * static_cast<double>(i) / static_cast<double>(N)));
  }
  return w;
}

}  // namespace itpms::ct
