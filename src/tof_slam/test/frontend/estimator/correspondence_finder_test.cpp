// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// correspondence_finder_test.cpp — Unit tests for find_correspondences_hybrid_select().

#include "tof_slam/frontend/estimator/correspondence_finder.hpp"

#include <cmath>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "tof_slam/common/lie/se3.hpp"
#include "tof_slam/common/lie/so3.hpp"
#include "tof_slam/frontend/map/surfel_map.hpp"
#include "tof_slam/frontend/map/point_voxel_map.hpp"
#include "tof_slam/common/types/point_types.hpp"
#include "tof_slam/common/types/state.hpp"

namespace tof_slam {
namespace core {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Build a PointCloud from a list of {x, y, z} triples.
static PointCloud make_cloud(const std::vector<std::array<float, 3>>& pts) {
  PointCloud cloud;
  cloud.reserve(pts.size());
  for (const auto& p : pts) {
    cloud.push_back(Point3D(p[0], p[1], p[2]));
  }
  return cloud;
}

/// Populate a SurfelMap with a flat XY plane at z = 0.
///
/// Inserts 25 points in a 5x5 grid within [0, 0.25)x[0, 0.25)x0, which
/// fall into distinct L0 voxels all inside a single L1 voxel (L1 = 0.3 m).
static void populate_flat_plane_map(SurfelMap& map) {
  std::vector<std::array<float, 3>> pts;
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 5; ++j) {
      // spacing 0.05 m → 25 distinct L0 voxels all in L1 voxel (0, 0, 0)
      pts.push_back({i * 0.05f + 0.02f, j * 0.05f + 0.02f, 0.0f});
    }
  }
  auto cloud = make_cloud(pts);
  map.update(cloud, Eigen::Vector3f::Zero());
}

/// Flat-plane SurfelMapConfig (reused across tests).
static SurfelMapConfig flat_plane_config() {
  SurfelMapConfig cfg;
  cfg.l0_voxel_size       = 0.1f;
  cfg.l1_hierarchy_factor = 3;   // L1 = 0.3 m
  cfg.planarity_threshold = 0.1f;
  cfg.min_l0_for_surfel   = 5;
  return cfg;
}

/// Default identity LioState.
static LioState make_identity_state() {
  LioState s;
  s.rotation = Eigen::Matrix3f::Identity();
  s.position = Eigen::Vector3f::Zero();
  return s;
}

/// Empty PointVoxelMap for tests that only exercise surfel fallback path.
static PointVoxelMap make_empty_pvmap() {
  PointVoxelMapConfig cfg;
  cfg.voxel_size = 0.5f;
  cfg.max_points_per_voxel = 20;
  return PointVoxelMap(cfg);
}

// ---------------------------------------------------------------------------
// EmptyMapReturnsEmpty
// ---------------------------------------------------------------------------

TEST(CorrespondenceFinderTest, EmptyMapReturnsEmpty) {
  SurfelMap empty_map(flat_plane_config());
  LioState state = make_identity_state();
  Se3 T_bl = Se3::Identity();

  PointCloud scan = make_cloud({{0.1f, 0.1f, 0.0f}});

  auto corrs = find_correspondences_hybrid_select(state, T_bl, scan, empty_map, make_empty_pvmap());
  EXPECT_TRUE(corrs.empty());
}

// ---------------------------------------------------------------------------
// EmptyScanReturnsEmpty
// ---------------------------------------------------------------------------

TEST(CorrespondenceFinderTest, EmptyScanReturnsEmpty) {
  SurfelMap map(flat_plane_config());
  populate_flat_plane_map(map);
  LioState state = make_identity_state();
  Se3 T_bl = Se3::Identity();

  PointCloud empty_scan;
  auto corrs = find_correspondences_hybrid_select(state, T_bl, empty_scan, map, make_empty_pvmap());
  EXPECT_TRUE(corrs.empty());
}

// ---------------------------------------------------------------------------
// FlatPlaneCorrespondences
// ---------------------------------------------------------------------------

TEST(CorrespondenceFinderTest, FlatPlaneCorrespondences) {
  SurfelMap map(flat_plane_config());
  populate_flat_plane_map(map);
  LioState state = make_identity_state();
  Se3 T_bl = Se3::Identity();

  // Scan points scattered on the z=0 plane inside the populated L1 voxel.
  PointCloud scan = make_cloud({
      {0.05f, 0.05f, 0.0f},
      {0.10f, 0.10f, 0.0f},
      {0.15f, 0.15f, 0.0f},
  });

  auto corrs = find_correspondences_hybrid_select(state, T_bl, scan, map, make_empty_pvmap());

  EXPECT_FALSE(corrs.empty())
      << "Expected correspondences for points on the flat z=0 plane";

  for (const auto& c : corrs) {
    // Normal must be approximately along Z axis.
    EXPECT_NEAR(std::abs(c.normal.z()), 1.0f, 0.05f)
        << "Normal z-component should be ~1 for flat XY-plane surfel";
  }
}

// ---------------------------------------------------------------------------
// CorrespondenceNormalsAreUnit
// ---------------------------------------------------------------------------

TEST(CorrespondenceFinderTest, CorrespondenceNormalsAreUnit) {
  SurfelMap map(flat_plane_config());
  populate_flat_plane_map(map);
  LioState state = make_identity_state();
  Se3 T_bl = Se3::Identity();

  PointCloud scan = make_cloud({
      {0.05f, 0.05f, 0.0f},
      {0.10f, 0.10f, 0.0f},
      {0.15f, 0.05f, 0.0f},
  });

  auto corrs = find_correspondences_hybrid_select(state, T_bl, scan, map, make_empty_pvmap());
  ASSERT_FALSE(corrs.empty());

  for (const auto& c : corrs) {
    EXPECT_NEAR(c.normal.norm(), 1.0f, 1e-4f)
        << "Surfel normal must be a unit vector";
  }
}

// ---------------------------------------------------------------------------
// PLidarIsInSensorFrame
// ---------------------------------------------------------------------------

TEST(CorrespondenceFinderTest, PLidarIsInSensorFrame) {
  SurfelMap map(flat_plane_config());
  populate_flat_plane_map(map);
  LioState state = make_identity_state();
  Se3 T_bl = Se3::Identity();

  // With identity state and identity extrinsic, world == lidar frame.
  const float px = 0.08f, py = 0.12f, pz = 0.0f;
  PointCloud scan = make_cloud({{px, py, pz}});

  auto corrs = find_correspondences_hybrid_select(state, T_bl, scan, map, make_empty_pvmap());
  ASSERT_FALSE(corrs.empty());

  EXPECT_NEAR(corrs[0].p_lidar.x(), px, 1e-5f);
  EXPECT_NEAR(corrs[0].p_lidar.y(), py, 1e-5f);
  EXPECT_NEAR(corrs[0].p_lidar.z(), pz, 1e-5f);
}

// ---------------------------------------------------------------------------
// WithExtrinsicTransform
// ---------------------------------------------------------------------------

TEST(CorrespondenceFinderTest, WithExtrinsicTransform) {
  // T_body_lidar: LiDAR is 0.1 m behind body along X.
  // Scan point at (0.2, 0.1, 0.0) in LiDAR frame maps to (0.1, 0.1, 0.0) in
  // world frame when identity state is used and T_bl translates by (-0.1, 0, 0).
  SurfelMap map(flat_plane_config());
  populate_flat_plane_map(map);
  LioState state = make_identity_state();

  // T_body_lidar: body = lidar + (−0.1, 0, 0)
  Se3 T_bl(Eigen::Matrix3f::Identity(),
           Eigen::Vector3f(-0.1f, 0.0f, 0.0f));

  // Scan point in LiDAR frame: (0.2, 0.1, 0.0)
  // Transformed to world: R_wb*(R_bl*(0.2,0.1,0)+(−0.1,0,0))+(0,0,0)
  //                     = (0.1, 0.1, 0.0)  → inside populated L1 voxel
  PointCloud scan = make_cloud({{0.2f, 0.1f, 0.0f}});

  auto corrs = find_correspondences_hybrid_select(state, T_bl, scan, map, make_empty_pvmap());
  EXPECT_FALSE(corrs.empty())
      << "Expected a correspondence after applying extrinsic transform";

  if (!corrs.empty()) {
    // p_lidar must be the original scan point, not the world-frame point.
    EXPECT_NEAR(corrs[0].p_lidar.x(), 0.2f, 1e-5f);
    EXPECT_NEAR(corrs[0].p_lidar.y(), 0.1f, 1e-5f);
    EXPECT_NEAR(corrs[0].p_lidar.z(), 0.0f, 1e-5f);
  }
}

// ---------------------------------------------------------------------------
// TranslatedState
// ---------------------------------------------------------------------------

TEST(CorrespondenceFinderTest, TranslatedState) {
  // Map has flat plane at z=0 in world frame.
  // State: robot is at position (-0.1, -0.1, 0) with identity rotation.
  // Scan point at (0.2, 0.2, 0.0) in LiDAR frame.
  // World frame point: (-0.1, -0.1, 0) + (0.2, 0.2, 0) = (0.1, 0.1, 0) - inside map.

  SurfelMap map(flat_plane_config());
  populate_flat_plane_map(map);
  LioState state = make_identity_state();
  state.position = Eigen::Vector3f(-0.1f, -0.1f, 0.0f);

  Se3 T_bl = Se3::Identity();
  PointCloud scan = make_cloud({{0.2f, 0.2f, 0.0f}});

  auto corrs = find_correspondences_hybrid_select(state, T_bl, scan, map, make_empty_pvmap());
  EXPECT_FALSE(corrs.empty())
      << "Expected correspondence when state has non-zero translation";

  if (!corrs.empty()) {
    // p_lidar must still be the original sensor-frame point.
    EXPECT_NEAR(corrs[0].p_lidar.x(), 0.2f, 1e-5f);
    EXPECT_NEAR(corrs[0].p_lidar.y(), 0.2f, 1e-5f);
  }
}

// ---------------------------------------------------------------------------
// PointOutsideMapReturnsNoCorrespondence
// ---------------------------------------------------------------------------

TEST(CorrespondenceFinderTest, PointOutsideMapReturnsNoCorrespondence) {
  SurfelMap map(flat_plane_config());
  populate_flat_plane_map(map);
  LioState state = make_identity_state();
  Se3 T_bl = Se3::Identity();

  // Point is 50 m away — far outside any populated L1 voxel.
  PointCloud scan = make_cloud({{50.0f, 50.0f, 0.0f}});

  auto corrs = find_correspondences_hybrid_select(state, T_bl, scan, map, make_empty_pvmap());
  EXPECT_TRUE(corrs.empty())
      << "Point far from the map should yield no correspondence";
}

}  // namespace core
}  // namespace tof_slam
