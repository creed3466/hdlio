// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// voxel_key.hpp — Integer voxel address and Morton (Z-order) hash.
//
// Shared between VoxelGrid (scan-level) and SurfelMap (map-level).

#ifndef TOF_SLAM_FRONTEND_MAP_VOXEL_KEY_HPP_
#define TOF_SLAM_FRONTEND_MAP_VOXEL_KEY_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>

namespace tof_slam {
namespace core {

// ---------------------------------------------------------------------------
// VoxelKey
// ---------------------------------------------------------------------------

struct VoxelKey {
  int x = 0;
  int y = 0;
  int z = 0;

  VoxelKey() = default;
  VoxelKey(int x, int y, int z) : x(x), y(y), z(z) {}

  bool operator==(const VoxelKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }

  bool operator!=(const VoxelKey& other) const { return !(*this == other); }

  bool operator<(const VoxelKey& other) const {
    if (x != other.x) return x < other.x;
    if (y != other.y) return y < other.y;
    return z < other.z;
  }
};

// ---------------------------------------------------------------------------
// Z-order (Morton code) hash — preserves spatial locality.
// ---------------------------------------------------------------------------

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& key) const {
    const uint64_t mx = expand_bits(key.x);
    const uint64_t my = expand_bits(key.y);
    const uint64_t mz = expand_bits(key.z);
    return static_cast<std::size_t>(mx | (my << 1) | (mz << 2));
  }

 private:
  /// Spread 21-bit integer into 63 bits with 2-bit gaps (for 3-D interleave).
  static uint64_t expand_bits(int32_t v) {
    uint64_t x = static_cast<uint64_t>(v + (1 << 20)) & 0x1fffffULL;
    x = (x | (x << 32)) & 0x1f00000000ffffULL;
    x = (x | (x << 16)) & 0x1f0000ff0000ffULL;
    x = (x | (x << 8))  & 0x100f00f00f00f00fULL;
    x = (x | (x << 4))  & 0x10c30c30c30c30c3ULL;
    x = (x | (x << 2))  & 0x1249249249249249ULL;
    return x;
  }
};

// ---------------------------------------------------------------------------
// Utility: point -> voxel key
// ---------------------------------------------------------------------------

/// Compute the integer voxel key for a point given voxel size.
/// Uses std::floor for correct negative-coordinate handling (matches reference).
inline VoxelKey point_to_voxel_key(float x, float y, float z,
                                   float inv_voxel_size) {
  return {static_cast<int>(std::floor(x * inv_voxel_size)),
          static_cast<int>(std::floor(y * inv_voxel_size)),
          static_cast<int>(std::floor(z * inv_voxel_size))};
}

}  // namespace core
}  // namespace tof_slam

#endif  // TOF_SLAM_FRONTEND_MAP_VOXEL_KEY_HPP_
