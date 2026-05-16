// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "tof_slam/frontend/filter/range_filter.hpp"
#include "tof_slam/frontend/filter/stride_filter.hpp"
#include "tof_slam/frontend/filter/voxel_grid.hpp"
#include "tof_slam/frontend/map/voxel_key.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <unordered_set>

namespace tof_slam {
namespace core {
namespace {

constexpr float kTol = 1e-5f;

// ===========================================================================
// VoxelKey and Morton hash
// ===========================================================================

TEST(VoxelKeyTest, Equality) {
  EXPECT_EQ(VoxelKey(1, 2, 3), VoxelKey(1, 2, 3));
  EXPECT_NE(VoxelKey(1, 2, 3), VoxelKey(1, 2, 4));
}

TEST(VoxelKeyTest, Ordering) {
  EXPECT_LT(VoxelKey(0, 0, 0), VoxelKey(1, 0, 0));
  EXPECT_LT(VoxelKey(1, 0, 0), VoxelKey(1, 1, 0));
}

TEST(VoxelKeyTest, MortonHashDeterministic) {
  VoxelKeyHash hasher;
  EXPECT_EQ(hasher(VoxelKey(1, 2, 3)), hasher(VoxelKey(1, 2, 3)));
}

TEST(VoxelKeyTest, MortonHashLowCollision) {
  VoxelKeyHash hasher;
  std::unordered_set<std::size_t> hashes;
  std::mt19937 rng(42);
  std::uniform_int_distribution<int> dist(-500, 500);
  for (int i = 0; i < 1000; ++i) {
    hashes.insert(hasher(VoxelKey(dist(rng), dist(rng), dist(rng))));
  }
  // Less than 5% collision rate.
  EXPECT_GT(hashes.size(), 950u);
}

TEST(VoxelKeyTest, PointToVoxelKey) {
  auto key = point_to_voxel_key(1.5f, 2.9f, -0.1f, 1.0f);  // voxel_size=1
  EXPECT_EQ(key.x, 1);
  EXPECT_EQ(key.y, 2);
  EXPECT_EQ(key.z, -1);  // floor(-0.1) = -1
}

TEST(VoxelKeyTest, PointToVoxelKeyNegative) {
  auto key = point_to_voxel_key(-1.5f, -2.9f, -0.01f, 2.0f);  // inv=2
  // -1.5 * 2 = -3.0 -> -3-1 = -4? No: -1.5 is negative, -1.5*2=-3.0 which is
  // exactly on boundary. floor_div: v<0 -> int(-3.0)-1 = -4
  // Actually: (int)(-3.0) = -3 on most platforms, then -3-1 = -4
  EXPECT_EQ(key.x, -4);
}

// ===========================================================================
// StrideFilter
// ===========================================================================

TEST(StrideFilterTest, Stride1IsIdentity) {
  PointCloud cloud;
  for (int i = 0; i < 10; ++i) {
    cloud.push_back(Point3D(static_cast<float>(i), 0, 0));
  }
  auto result = stride_filter(cloud, 1);
  EXPECT_EQ(result.size(), 10u);
}

TEST(StrideFilterTest, Stride4) {
  PointCloud cloud;
  for (int i = 0; i < 100; ++i) {
    cloud.push_back(Point3D(static_cast<float>(i), 0, 0));
  }
  auto result = stride_filter(cloud, 4);
  EXPECT_EQ(result.size(), 25u);
  EXPECT_FLOAT_EQ(result[0].x, 0.0f);
  EXPECT_FLOAT_EQ(result[1].x, 4.0f);
}

TEST(StrideFilterTest, StrideGreaterThanSize) {
  PointCloud cloud;
  cloud.push_back(Point3D(1, 2, 3));
  auto result = stride_filter(cloud, 100);
  EXPECT_EQ(result.size(), 1u);
}

TEST(StrideFilterTest, EmptyCloud) {
  PointCloud cloud;
  auto result = stride_filter(cloud, 4);
  EXPECT_TRUE(result.empty());
}

TEST(StrideFilterTest, InvalidStride) {
  PointCloud cloud;
  cloud.push_back(Point3D(1, 0, 0));
  EXPECT_TRUE(stride_filter(cloud, 0).empty());
  EXPECT_TRUE(stride_filter(cloud, -1).empty());
}

// ===========================================================================
// RangeFilter
// ===========================================================================

TEST(RangeFilterTest, BasicFiltering) {
  PointCloud cloud;
  cloud.push_back(Point3D(0.1f, 0, 0));    // range 0.1
  cloud.push_back(Point3D(1.0f, 0, 0));    // range 1.0
  cloud.push_back(Point3D(200.0f, 0, 0));  // range 200

  auto result = range_filter(cloud, 0.5f, 100.0f);
  EXPECT_EQ(result.size(), 1u);
  EXPECT_NEAR(result[0].x, 1.0f, kTol);
}

TEST(RangeFilterTest, AllOutsideRange) {
  PointCloud cloud;
  cloud.push_back(Point3D(0.01f, 0, 0));
  cloud.push_back(Point3D(0.02f, 0, 0));

  auto result = range_filter(cloud, 1.0f, 10.0f);
  EXPECT_TRUE(result.empty());
}

TEST(RangeFilterTest, EmptyInput) {
  PointCloud cloud;
  auto result = range_filter(cloud, 0.0f, 100.0f);
  EXPECT_TRUE(result.empty());
}

TEST(RangeFilterTest, InvalidRange) {
  PointCloud cloud;
  cloud.push_back(Point3D(1, 0, 0));
  auto result = range_filter(cloud, 10.0f, 1.0f);  // min > max
  EXPECT_TRUE(result.empty());
}

TEST(RangeFilterTest, 3DDistance) {
  PointCloud cloud;
  cloud.push_back(Point3D(3.0f, 4.0f, 0.0f));  // range = 5.0
  auto result = range_filter(cloud, 4.0f, 6.0f);
  EXPECT_EQ(result.size(), 1u);
}

// ===========================================================================
// VoxelGrid
// ===========================================================================

TEST(VoxelGridTest, BasicDownsampling) {
  PointCloud cloud;
  // 8 points all in the same voxel (leaf=1.0)
  for (int i = 0; i < 8; ++i) {
    cloud.push_back(Point3D(0.1f * static_cast<float>(i), 0, 0));
  }

  VoxelGridConfig cfg;
  cfg.leaf_size = 1.0f;
  auto result = voxel_grid_filter(cloud, cfg);
  EXPECT_EQ(result.size(), 1u);
}

TEST(VoxelGridTest, MultipleVoxels) {
  PointCloud cloud;
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> dist(0.0f, 10.0f);
  for (int i = 0; i < 1000; ++i) {
    cloud.push_back(Point3D(dist(rng), dist(rng), dist(rng)));
  }

  VoxelGridConfig cfg;
  cfg.leaf_size = 1.0f;
  auto result = voxel_grid_filter(cloud, cfg);
  EXPECT_LT(result.size(), cloud.size());
  EXPECT_GT(result.size(), 0u);
  // 10x10x10 grid with leaf=1 -> at most 1000 voxels, but likely ~600-900
  EXPECT_LE(result.size(), 1000u);
}

TEST(VoxelGridTest, SinglePoint) {
  PointCloud cloud;
  cloud.push_back(Point3D(5.0f, 5.0f, 5.0f));

  VoxelGridConfig cfg;
  cfg.leaf_size = 1.0f;
  auto result = voxel_grid_filter(cloud, cfg);
  EXPECT_EQ(result.size(), 1u);
}

TEST(VoxelGridTest, EmptyInput) {
  PointCloud cloud;
  VoxelGridConfig cfg;
  auto result = voxel_grid_filter(cloud, cfg);
  EXPECT_TRUE(result.empty());
}

TEST(VoxelGridTest, AllIdenticalPoints) {
  PointCloud cloud;
  for (int i = 0; i < 100; ++i) {
    cloud.push_back(Point3D(1.0f, 2.0f, 3.0f));
  }

  VoxelGridConfig cfg;
  cfg.leaf_size = 0.5f;
  auto result = voxel_grid_filter(cloud, cfg);
  EXPECT_EQ(result.size(), 1u);
  EXPECT_NEAR(result[0].x, 1.0f, kTol);
}

TEST(VoxelGridTest, CentroidCorrectness) {
  PointCloud cloud;
  cloud.push_back(Point3D(0.0f, 0.0f, 0.0f));
  cloud.push_back(Point3D(0.4f, 0.4f, 0.4f));  // Same voxel with leaf=1

  VoxelGridConfig cfg;
  cfg.leaf_size = 1.0f;
  auto result = voxel_grid_filter(cloud, cfg);
  EXPECT_EQ(result.size(), 1u);
  EXPECT_NEAR(result[0].x, 0.2f, kTol);
  EXPECT_NEAR(result[0].y, 0.2f, kTol);
}

TEST(VoxelGridTest, PlanarityFilterKeepsPlanarVoxels) {
  PointCloud cloud;
  // Flat plane at z=0 (should pass planarity filter)
  for (int i = 0; i < 20; ++i) {
    for (int j = 0; j < 20; ++j) {
      cloud.push_back(Point3D(
          static_cast<float>(i) * 0.04f,
          static_cast<float>(j) * 0.04f,
          0.0f));
    }
  }

  VoxelGridConfig cfg;
  cfg.leaf_size = 1.0f;
  cfg.enable_planarity_filter = true;
  cfg.planarity_threshold = 0.1f;
  auto result = voxel_grid_filter(cloud, cfg);
  // One voxel, planar -> should pass
  EXPECT_GE(result.size(), 1u);
}

TEST(VoxelGridTest, IntensityPreserved) {
  PointCloud cloud;
  cloud.push_back(Point3D(0, 0, 0, 10.0f, 0.1f));
  cloud.push_back(Point3D(0.1f, 0, 0, 20.0f, 0.3f));

  VoxelGridConfig cfg;
  cfg.leaf_size = 1.0f;
  auto result = voxel_grid_filter(cloud, cfg);
  EXPECT_EQ(result.size(), 1u);
  EXPECT_NEAR(result[0].intensity, 15.0f, kTol);
  EXPECT_NEAR(result[0].offset_time, 0.2f, kTol);
}

}  // namespace
}  // namespace core
}  // namespace tof_slam
