#pragma once

/// Hilbert curve position-to-index encoding (transposed form).
/// Based on "Programming the Hilbert Curve" by John Skilling.
/// Ported from spectral3d/hilbert_hpp (MIT license).

#include <array>
#include <cstdint>

namespace tof_slam {
namespace hilbert {

/// Convert N-dimensional position to Hilbert curve index (transposed form).
/// Input:  N coordinate values, each in [0, 2^(8*sizeof(T))).
/// Output: N transposed index values; combine as
///         idx = (out[0] << 16) | (out[1] << 8) | out[2]  for N=3, T=uint8_t.
template <typename T, size_t N>
std::array<T, N> positionToIndex(std::array<T, N> X) {
  const T M = T(1) << (8 * sizeof(T) - 1);

  // Inverse undo excess work
  for (T Q = M; Q > 1; Q >>= 1) {
    const T P = Q - 1;
    for (size_t i = 0; i < N; ++i) {
      if (X[i] & Q) {
        X[0] ^= P;
      } else {
        const T t = (X[0] ^ X[i]) & P;
        X[0] ^= t;
        X[i] ^= t;
      }
    }
  }

  // Gray encode
  for (size_t i = 1; i < N; ++i) {
    X[i] ^= X[i - 1];
  }
  T t = 0;
  for (T Q = M; Q > 1; Q >>= 1) {
    if (X[N - 1] & Q) {
      t ^= (Q - 1);
    }
  }
  for (size_t i = 0; i < N; ++i) {
    X[i] ^= t;
  }

  return X;
}

/// Convert Hilbert curve index (transposed form) back to N-dimensional position.
template <typename T, size_t N>
std::array<T, N> indexToPosition(std::array<T, N> X) {
  const int n = static_cast<int>(8 * sizeof(T));

  // Gray decode by H^{-1} = (Gray XOR) / 2
  T t = X[N - 1] >> 1;
  for (size_t i = N - 1; i > 0; --i) {
    X[i] ^= X[i - 1];
  }
  X[0] ^= t;

  // Undo excess work
  // Use int loop counter to avoid uint8_t overflow (T(1)<<n wraps to 0).
  for (int bit = 1; bit < n; ++bit) {
    const T Q = T(1) << bit;
    const T P = Q - 1;
    for (int i = static_cast<int>(N) - 1; i >= 0; --i) {
      if (X[i] & Q) {
        X[0] ^= P;
      } else {
        const T t2 = (X[0] ^ X[i]) & P;
        X[0] ^= t2;
        X[i] ^= t2;
      }
    }
  }

  return X;
}

}  // namespace hilbert
}  // namespace tof_slam
