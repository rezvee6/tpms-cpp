// Lock-free single-producer / single-consumer ring. The producer is the
// input-capture ISR, the consumer is the background task. Free-running
// 32-bit indices: N divides 2^32, so wrap-around needs no special case.
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "itpms/ct_math.hpp"

namespace itpms {

template <typename T, std::size_t N>
class SpscRing {
  static_assert(ct::is_pow2(N), "capacity must be a power of two");
  static_assert(std::atomic<std::uint32_t>::is_always_lock_free, "ISR-safe ring needs lock-free atomics");

 public:
  bool push(const T& v) noexcept {
    const std::uint32_t head = head_.load(std::memory_order_relaxed);
    if (head - tail_.load(std::memory_order_acquire) == N) return false;
    buf_[head & kMask] = v;
    head_.store(head + 1, std::memory_order_release);
    return true;
  }

  bool pop(T& out) noexcept {
    const std::uint32_t tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) return false;
    out = buf_[tail & kMask];
    tail_.store(tail + 1, std::memory_order_release);
    return true;
  }

  std::size_t size() const noexcept {
    return head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire);
  }
  static constexpr std::size_t capacity() noexcept { return N; }

 private:
  static constexpr std::uint32_t kMask = static_cast<std::uint32_t>(N - 1);
  std::array<T, N> buf_{};
  std::atomic<std::uint32_t> head_{0};
  std::atomic<std::uint32_t> tail_{0};
};

}  // namespace itpms
