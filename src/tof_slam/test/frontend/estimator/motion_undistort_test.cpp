// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// motion_undistort_test.cpp — Unit tests for motion undistortion.

#include "tof_slam/frontend/estimator/motion_undistort.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace tof_slam {
namespace core {
namespace {

// Tolerance for floating-point comparisons.
constexpr float kTol = 1e-4f;

// Helper: build a StampedPose from raw rotation matrix + translation.
StampedPose make_stamped(double t, const Eigen::Matrix3f& R,
                          const Eigen::Vector3f& p) {
  StampedPose sp;
  sp.timestamp = t;
  sp.pose      = Se3(R, p);
  return sp;
}

// Helper: build a StampedPose with identity rotation and given translation.
StampedPose make_translation(double t, float tx, float ty, float tz) {
  return make_stamped(t, Eigen::Matrix3f::Identity(),
                      Eigen::Vector3f(tx, ty, tz));
}

// Helper: build a StampedPose with identity transform.
StampedPose make_identity(double t) {
  return make_stamped(t, Eigen::Matrix3f::Identity(),
                      Eigen::Vector3f::Zero());
}

// Helper: rotation around Z-axis by angle_deg degrees.
Eigen::Matrix3f rot_z(float angle_deg) {
  const float a = angle_deg * static_cast<float>(M_PI) / 180.0f;
  Eigen::Matrix3f R;
  R << std::cos(a), -std::sin(a), 0.0f, std::sin(a), std::cos(a), 0.0f, 0.0f,
      0.0f, 1.0f;
  return R;
}

// ===========================================================================
// interpolate_pose tests
// ===========================================================================

TEST(MotionUndistortTest, InterpolatePoseIdentity) {
  const Se3 I = Se3::Identity();
  const Se3 result = interpolate_pose(I, I, 0.5f);

  // Identity interpolated at any alpha should remain identity.
  EXPECT_NEAR(result.translation().x(), 0.0f, kTol);
  EXPECT_NEAR(result.translation().y(), 0.0f, kTol);
  EXPECT_NEAR(result.translation().z(), 0.0f, kTol);
  const Eigen::Matrix3f diff =
      result.rotation_matrix() - Eigen::Matrix3f::Identity();
  EXPECT_NEAR(diff.norm(), 0.0f, kTol);
}

TEST(MotionUndistortTest, InterpolatePoseEndpointAlpha0) {
  const Se3 A(Eigen::Matrix3f::Identity(), Eigen::Vector3f(1.0f, 2.0f, 3.0f));
  const Se3 B(Eigen::Matrix3f::Identity(), Eigen::Vector3f(5.0f, 6.0f, 7.0f));

  const Se3 result = interpolate_pose(A, B, 0.0f);

  EXPECT_NEAR(result.translation().x(), 1.0f, kTol);
  EXPECT_NEAR(result.translation().y(), 2.0f, kTol);
  EXPECT_NEAR(result.translation().z(), 3.0f, kTol);
}

TEST(MotionUndistortTest, InterpolatePoseEndpointAlpha1) {
  const Se3 A(Eigen::Matrix3f::Identity(), Eigen::Vector3f(1.0f, 2.0f, 3.0f));
  const Se3 B(Eigen::Matrix3f::Identity(), Eigen::Vector3f(5.0f, 6.0f, 7.0f));

  const Se3 result = interpolate_pose(A, B, 1.0f);

  EXPECT_NEAR(result.translation().x(), 5.0f, kTol);
  EXPECT_NEAR(result.translation().y(), 6.0f, kTol);
  EXPECT_NEAR(result.translation().z(), 7.0f, kTol);
}

TEST(MotionUndistortTest, InterpolatePoseMiddlePureTranslation) {
  // Interpolating pure translation: [0,0,0] to [2,0,0] at alpha=0.5 -> [1,0,0]
  const Se3 A(Eigen::Matrix3f::Identity(), Eigen::Vector3f(0.0f, 0.0f, 0.0f));
  const Se3 B(Eigen::Matrix3f::Identity(), Eigen::Vector3f(2.0f, 0.0f, 0.0f));

  const Se3 result = interpolate_pose(A, B, 0.5f);

  EXPECT_NEAR(result.translation().x(), 1.0f, kTol);
  EXPECT_NEAR(result.translation().y(), 0.0f, kTol);
  EXPECT_NEAR(result.translation().z(), 0.0f, kTol);
}

TEST(MotionUndistortTest, InterpolatePoseRotation) {
  // Pure rotation around Z: 0 deg to 90 deg, alpha=0.5 should give ~45 deg.
  const Se3 A(Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero());
  const Se3 B(rot_z(90.0f), Eigen::Vector3f::Zero());

  const Se3 result = interpolate_pose(A, B, 0.5f);
  const Se3 expected(rot_z(45.0f), Eigen::Vector3f::Zero());

  const Eigen::Matrix3f diff =
      result.rotation_matrix() - expected.rotation_matrix();
  EXPECT_NEAR(diff.norm(), 0.0f, kTol);
}

// ===========================================================================
// undistort_scan edge-case tests
// ===========================================================================

TEST(MotionUndistortTest, EmptyTrajectory) {
  PointCloud cloud;
  cloud.push_back(Point3D(1.0f, 2.0f, 3.0f, 0.5f, -0.05f));

  const std::vector<StampedPose> traj;  // empty
  const Se3 T_bl = Se3::Identity();

  const PointCloud result = undistort_scan(cloud, traj, 1.0, T_bl);

  // Returns raw cloud unchanged.
  ASSERT_EQ(result.size(), cloud.size());
  EXPECT_NEAR(result[0].x, 1.0f, kTol);
  EXPECT_NEAR(result[0].y, 2.0f, kTol);
  EXPECT_NEAR(result[0].z, 3.0f, kTol);
  EXPECT_NEAR(result[0].intensity, 0.5f, kTol);
  EXPECT_NEAR(result[0].offset_time, -0.05f, kTol);
}

TEST(MotionUndistortTest, SingleTrajectoryEntry) {
  PointCloud cloud;
  cloud.push_back(Point3D(1.0f, 2.0f, 3.0f, 0.5f, -0.05f));

  std::vector<StampedPose> traj;
  traj.push_back(make_identity(1.0));  // single entry

  const Se3 T_bl = Se3::Identity();

  const PointCloud result = undistort_scan(cloud, traj, 1.0, T_bl);

  // Cannot interpolate with a single entry — returns raw cloud unchanged.
  ASSERT_EQ(result.size(), cloud.size());
  EXPECT_NEAR(result[0].x, 1.0f, kTol);
  EXPECT_NEAR(result[0].y, 2.0f, kTol);
  EXPECT_NEAR(result[0].z, 3.0f, kTol);
  EXPECT_NEAR(result[0].offset_time, -0.05f, kTol);
}

TEST(MotionUndistortTest, StationaryRobot) {
  // All trajectory poses identical (robot not moving).
  // Undistorted cloud should equal original cloud (modulo offset_time reset).
  PointCloud cloud;
  cloud.push_back(Point3D(1.0f, 0.0f, 0.0f, 1.0f, -0.05f));
  cloud.push_back(Point3D(0.0f, 1.0f, 0.0f, 2.0f, -0.02f));
  cloud.push_back(Point3D(0.0f, 0.0f, 1.0f, 3.0f,  0.00f));

  const double t_end = 1.0;
  std::vector<StampedPose> traj;
  traj.push_back(make_identity(t_end - 0.1));
  traj.push_back(make_identity(t_end));

  const Se3 T_bl = Se3::Identity();

  const PointCloud result = undistort_scan(cloud, traj, t_end, T_bl);

  ASSERT_EQ(result.size(), cloud.size());
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    EXPECT_NEAR(result[i].x, cloud[i].x, kTol) << "Point " << i;
    EXPECT_NEAR(result[i].y, cloud[i].y, kTol) << "Point " << i;
    EXPECT_NEAR(result[i].z, cloud[i].z, kTol) << "Point " << i;
    EXPECT_NEAR(result[i].intensity, cloud[i].intensity, kTol) << "Point " << i;
    EXPECT_NEAR(result[i].offset_time, 0.0f, kTol) << "Point " << i;
  }
}

TEST(MotionUndistortTest, PureTranslationMidpoint) {
  // Robot moves 1 m along X over 0.1 s. A point at offset_time=-0.05
  // (midpoint) should be corrected toward the end-frame.
  //
  // Setup:
  //   traj[0] at t=0.9: position=[0,0,0]
  //   traj[1] at t=1.0: position=[1,0,0]  <- scan_end_time
  //
  // A point at scan body origin at capture time t=0.95 (midpoint) is
  // at world position [0.5, 0, 0].  Projecting to end frame (body at [1,0,0])
  // gives body position [-0.5, 0, 0].
  const double t_end = 1.0;

  std::vector<StampedPose> traj;
  traj.push_back(make_translation(0.9, 0.0f, 0.0f, 0.0f));
  traj.push_back(make_translation(1.0, 1.0f, 0.0f, 0.0f));

  // Place a point at the body/lidar origin at capture time.
  // With identity extrinsic, a point at [0,0,0] in lidar == body frame.
  PointCloud cloud;
  cloud.push_back(Point3D(0.0f, 0.0f, 0.0f, 1.0f, -0.05f));

  const Se3 T_bl = Se3::Identity();

  const PointCloud result = undistort_scan(cloud, traj, t_end, T_bl);

  ASSERT_EQ(result.size(), 1u);
  // At capture (t=0.95), body is at world [0.5,0,0].
  // At end    (t=1.0),  body is at world [1.0,0,0].
  // World point p_world = T_point * [0,0,0] = [0.5,0,0].
  // In end body frame: T_end_inv * p_world = [0.5,0,0] - [1,0,0] = [-0.5,0,0].
  EXPECT_NEAR(result[0].x, -0.5f, kTol);
  EXPECT_NEAR(result[0].y,  0.0f, kTol);
  EXPECT_NEAR(result[0].z,  0.0f, kTol);
  EXPECT_NEAR(result[0].offset_time, 0.0f, kTol);
}

TEST(MotionUndistortTest, PreservesIntensity) {
  const double t_end = 1.0;
  std::vector<StampedPose> traj;
  traj.push_back(make_identity(0.9));
  traj.push_back(make_identity(1.0));

  PointCloud cloud;
  cloud.push_back(Point3D(1.0f, 0.0f, 0.0f, 42.0f, -0.03f));
  cloud.push_back(Point3D(0.0f, 1.0f, 0.0f, 99.5f, -0.07f));

  const Se3 T_bl = Se3::Identity();
  const PointCloud result = undistort_scan(cloud, traj, t_end, T_bl);

  ASSERT_EQ(result.size(), cloud.size());
  EXPECT_NEAR(result[0].intensity, 42.0f, kTol);
  EXPECT_NEAR(result[1].intensity, 99.5f, kTol);
}

TEST(MotionUndistortTest, PointCountPreserved) {
  const double t_end = 1.0;
  std::vector<StampedPose> traj;
  traj.push_back(make_translation(0.8, 0.0f, 0.0f, 0.0f));
  traj.push_back(make_translation(1.0, 2.0f, 0.0f, 0.0f));

  PointCloud cloud;
  for (int i = 0; i < 100; ++i) {
    cloud.push_back(
        Point3D(static_cast<float>(i), 0.0f, 0.0f, 1.0f,
                static_cast<float>(-0.1 * i / 100.0)));
  }

  const Se3 T_bl = Se3::Identity();
  const PointCloud result = undistort_scan(cloud, traj, t_end, T_bl);

  EXPECT_EQ(result.size(), cloud.size());
}

TEST(MotionUndistortTest, OffsetTimeResetToZero) {
  const double t_end = 1.0;
  std::vector<StampedPose> traj;
  traj.push_back(make_identity(0.9));
  traj.push_back(make_identity(1.0));

  PointCloud cloud;
  cloud.push_back(Point3D(1.0f, 2.0f, 3.0f, 0.0f, -0.07f));
  cloud.push_back(Point3D(4.0f, 5.0f, 6.0f, 0.0f, -0.01f));

  const Se3 T_bl = Se3::Identity();
  const PointCloud result = undistort_scan(cloud, traj, t_end, T_bl);

  ASSERT_EQ(result.size(), 2u);
  EXPECT_NEAR(result[0].offset_time, 0.0f, kTol);
  EXPECT_NEAR(result[1].offset_time, 0.0f, kTol);
}

}  // namespace
}  // namespace core
}  // namespace tof_slam
