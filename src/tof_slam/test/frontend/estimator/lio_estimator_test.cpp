// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// lio_estimator_test.cpp — Unit tests for the LioEstimator orchestrator.

#include "tof_slam/frontend/estimator/lio_estimator.hpp"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>
#include <thread>
#include <vector>

#include "tof_slam/common/types/imu_types.hpp"
#include "tof_slam/common/types/point_types.hpp"
#include "tof_slam/common/types/state.hpp"

namespace tof_slam {
namespace core {
namespace {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Generate a buffer of stationary IMU samples at a given timestamp cadence.
/// Accelerometer reads upward (positive z) at 9.81 m/s^2, gyro is zero.
std::vector<ImuMeasurement> make_stationary_imu_buffer(
    int n, double t0 = 0.0, double dt = 0.005) {
  std::vector<ImuMeasurement> buf;
  buf.reserve(n);
  for (int i = 0; i < n; ++i) {
    ImuMeasurement m;
    m.timestamp = t0 + i * dt;
    m.accel = Eigen::Vector3f(0.0f, 0.0f, 9.81f);  // upward (sensor at rest)
    m.gyro = Eigen::Vector3f::Zero();
    buf.push_back(m);
  }
  return buf;
}

/// Generate a flat horizontal plane at z = 0, centred around (cx, cy).
/// Points are placed in a grid pattern at the given height in LiDAR frame.
PointCloud make_flat_plane_cloud(int n_side, float spacing, float height,
                                  float cx = 0.0f, float cy = 0.0f) {
  PointCloud cloud;
  for (int i = 0; i < n_side; ++i) {
    for (int j = 0; j < n_side; ++j) {
      float x = cx + (i - n_side / 2) * spacing;
      float y = cy + (j - n_side / 2) * spacing;
      cloud.push_back(Point3D(x, y, height));
    }
  }
  return cloud;
}

/// Make a LioEstimator with relaxed surfel requirements for testing.
LioEstimator::Config make_test_config() {
  LioEstimator::Config cfg;
  cfg.stride = 1;              // no stride skipping in tests
  cfg.voxel_leaf_size = 0.5f;  // moderate voxel size
  cfg.min_range = 0.1f;
  cfg.max_range = 100.0f;
  cfg.enable_undistortion = false;

  // Surfel map: relax requirements for small test clouds
  cfg.surfel_map.l0_voxel_size = 0.5f;
  cfg.surfel_map.l1_hierarchy_factor = 3;
  cfg.surfel_map.min_l0_for_surfel = 3;
  cfg.surfel_map.planarity_threshold = 0.1f;
  cfg.surfel_map.max_distance = 100.0f;

  // IEKF
  cfg.iekf.max_inner_iters = 5;
  cfg.iekf.max_outer_iters = 3;
  cfg.iekf.convergence_threshold = 1e-4f;
  cfg.iekf.lidar_noise_std = 0.01f;

  // PKO: disable adaptive for deterministic tests
  cfg.pko.use_adaptive = false;

  return cfg;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

TEST(LioEstimatorTest, DefaultConstruction) {
  LioEstimator est;
  EXPECT_FALSE(est.initialized());
  EXPECT_EQ(est.frame_count(), 0);
}

TEST(LioEstimatorTest, InitializeWithIMU) {
  auto cfg = make_test_config();
  LioEstimator est(cfg);

  auto buf = make_stationary_imu_buffer(100);
  EXPECT_TRUE(est.initialize(buf));
  EXPECT_TRUE(est.initialized());

  // State should have gravity roughly aligned to [0, 0, -9.81].
  LioState s = est.current_state();
  EXPECT_NEAR(s.gravity.z(), -kGravityNorm, 0.5f);
}

TEST(LioEstimatorTest, InitializeFailsWithFewSamples) {
  LioEstimator est;
  auto buf = make_stationary_imu_buffer(5);  // too few
  EXPECT_FALSE(est.initialize(buf));
  EXPECT_FALSE(est.initialized());
}

TEST(LioEstimatorTest, InitializeFailsWhenAlreadyInitialized) {
  auto cfg = make_test_config();
  LioEstimator est(cfg);

  auto buf = make_stationary_imu_buffer(100);
  EXPECT_TRUE(est.initialize(buf));
  EXPECT_FALSE(est.initialize(buf));  // second call should fail
}

TEST(LioEstimatorTest, FeedImuBeforeInit) {
  LioEstimator est;
  ImuMeasurement imu;
  imu.timestamp = 1.0;
  imu.accel = Eigen::Vector3f(0.0f, 0.0f, 9.81f);
  imu.gyro = Eigen::Vector3f::Zero();

  // Should not crash.
  est.feed_imu(imu);
  EXPECT_FALSE(est.initialized());
}

TEST(LioEstimatorTest, FeedLidarBeforeInit) {
  LioEstimator est;
  PointCloud cloud;
  cloud.push_back(Point3D(1.0f, 0.0f, 0.0f));

  // Should not crash, should return false.
  EXPECT_FALSE(est.feed_lidar(cloud, 1.0));
}

TEST(LioEstimatorTest, FeedImuPropagates) {
  auto cfg = make_test_config();
  LioEstimator est(cfg);

  auto buf = make_stationary_imu_buffer(100, 0.0, 0.005);
  ASSERT_TRUE(est.initialize(buf));

  LioState before = est.current_state();

  // Feed IMU with some forward acceleration for a known dt.
  ImuMeasurement imu;
  imu.timestamp = buf.back().timestamp + 0.01;
  imu.accel = Eigen::Vector3f(1.0f, 0.0f, 9.81f);  // 1 m/s^2 forward
  imu.gyro = Eigen::Vector3f::Zero();
  est.feed_imu(imu);

  LioState after = est.current_state();

  // Velocity or position should have changed.
  float state_change = (after.velocity - before.velocity).norm() +
                        (after.position - before.position).norm();
  EXPECT_GT(state_change, 0.0f);
}

TEST(LioEstimatorTest, FeedLidarFirstFrame) {
  auto cfg = make_test_config();
  LioEstimator est(cfg);

  auto buf = make_stationary_imu_buffer(100, 0.0, 0.005);
  ASSERT_TRUE(est.initialize(buf));

  // Create a reasonably sized flat plane cloud for the map to populate.
  PointCloud cloud = make_flat_plane_cloud(20, 0.3f, -1.0f);

  bool result = est.feed_lidar(cloud, 1.0);
  EXPECT_TRUE(result);
  EXPECT_EQ(est.frame_count(), 1);

  // Map should now have some points.
  EXPECT_GT(est.surfel_map().point_count(), 0u);
}

TEST(LioEstimatorTest, FeedLidarSecondFrame) {
  auto cfg = make_test_config();
  LioEstimator est(cfg);

  auto buf = make_stationary_imu_buffer(100, 0.0, 0.005);
  ASSERT_TRUE(est.initialize(buf));

  // First frame: populate map with a large flat plane.
  PointCloud cloud1 = make_flat_plane_cloud(30, 0.3f, -1.0f);
  ASSERT_TRUE(est.feed_lidar(cloud1, 1.0));

  // Feed a few IMU samples between frames.
  double t_imu = 1.0;
  for (int i = 0; i < 20; ++i) {
    t_imu += 0.005;
    ImuMeasurement imu;
    imu.timestamp = t_imu;
    imu.accel = Eigen::Vector3f(0.0f, 0.0f, 9.81f);
    imu.gyro = Eigen::Vector3f::Zero();
    est.feed_imu(imu);
  }

  // Second frame: IEKF should run.
  PointCloud cloud2 = make_flat_plane_cloud(30, 0.3f, -1.0f);
  bool result = est.feed_lidar(cloud2, t_imu + 0.005);
  EXPECT_TRUE(result);
  EXPECT_EQ(est.frame_count(), 2);
}

TEST(LioEstimatorTest, FullPipeline) {
  auto cfg = make_test_config();
  LioEstimator est(cfg);

  // Initialize
  auto buf = make_stationary_imu_buffer(100, 0.0, 0.005);
  ASSERT_TRUE(est.initialize(buf));

  double t = buf.back().timestamp;

  // Feed IMU samples
  for (int i = 0; i < 50; ++i) {
    t += 0.005;
    ImuMeasurement imu;
    imu.timestamp = t;
    imu.accel = Eigen::Vector3f(0.0f, 0.0f, 9.81f);
    imu.gyro = Eigen::Vector3f::Zero();
    est.feed_imu(imu);
  }

  // Feed first LiDAR frame
  PointCloud cloud = make_flat_plane_cloud(30, 0.3f, -1.0f);
  t += 0.005;
  ASSERT_TRUE(est.feed_lidar(cloud, t));

  // Feed more IMU
  for (int i = 0; i < 50; ++i) {
    t += 0.005;
    ImuMeasurement imu;
    imu.timestamp = t;
    imu.accel = Eigen::Vector3f(0.0f, 0.0f, 9.81f);
    imu.gyro = Eigen::Vector3f::Zero();
    est.feed_imu(imu);
  }

  // Feed second LiDAR frame
  t += 0.005;
  ASSERT_TRUE(est.feed_lidar(cloud, t));

  // State should remain reasonably close to origin (stationary scenario).
  // Tolerance is generous because IMU integration drift accumulates between
  // LiDAR frames, and the IEKF correction depends on map quality.
  LioState s = est.current_state();
  EXPECT_NEAR(s.position.norm(), 0.0f, 5.0f);
  EXPECT_NEAR(s.velocity.norm(), 0.0f, 5.0f);
}

TEST(LioEstimatorTest, Reset) {
  auto cfg = make_test_config();
  LioEstimator est(cfg);

  auto buf = make_stationary_imu_buffer(100);
  ASSERT_TRUE(est.initialize(buf));
  EXPECT_TRUE(est.initialized());

  est.reset();
  EXPECT_FALSE(est.initialized());
  EXPECT_EQ(est.frame_count(), 0);
}

TEST(LioEstimatorTest, ThreadSafety) {
  auto cfg = make_test_config();
  LioEstimator est(cfg);

  auto buf = make_stationary_imu_buffer(100, 0.0, 0.005);
  ASSERT_TRUE(est.initialize(buf));

  // Read current_state from another thread while feeding IMU.
  std::atomic<bool> done{false};
  std::thread reader([&]() {
    while (!done.load()) {
      LioState s = est.current_state();
      (void)s;  // Just exercise the thread-safe accessor.
    }
  });

  double t = buf.back().timestamp;
  for (int i = 0; i < 100; ++i) {
    t += 0.005;
    ImuMeasurement imu;
    imu.timestamp = t;
    imu.accel = Eigen::Vector3f(0.0f, 0.0f, 9.81f);
    imu.gyro = Eigen::Vector3f::Zero();
    est.feed_imu(imu);
  }

  done.store(true);
  reader.join();

  // If we get here without crashing or deadlocking, the test passes.
  EXPECT_TRUE(est.initialized());
}

TEST(LioEstimatorTest, MapSurfelsAccessible) {
  auto cfg = make_test_config();
  LioEstimator est(cfg);

  auto buf = make_stationary_imu_buffer(100, 0.0, 0.005);
  ASSERT_TRUE(est.initialize(buf));

  // Before any lidar, surfels should be empty.
  auto surfels = est.map_surfels();
  EXPECT_TRUE(surfels.empty());

  // After adding a large plane cloud, surfels may appear.
  PointCloud cloud = make_flat_plane_cloud(30, 0.3f, -1.0f);
  est.feed_lidar(cloud, 1.0);

  // There should be some surfel data now (point_count > 0 at minimum).
  EXPECT_GT(est.surfel_map().point_count(), 0u);
}

TEST(LioEstimatorTest, EmptyScanSkipped) {
  auto cfg = make_test_config();
  LioEstimator est(cfg);

  auto buf = make_stationary_imu_buffer(100, 0.0, 0.005);
  ASSERT_TRUE(est.initialize(buf));

  PointCloud empty_cloud;
  EXPECT_FALSE(est.feed_lidar(empty_cloud, 1.0));
  EXPECT_EQ(est.frame_count(), 0);
}

}  // namespace
}  // namespace core
}  // namespace tof_slam
