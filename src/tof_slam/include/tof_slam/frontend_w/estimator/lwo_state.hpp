// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// lwo_state.hpp — 15-dimensional LWO state (LiDAR-Wheel Odometry, no IMU).
//
// Layout: [rotation(3) | position(3) | velocity(3) |
//          wheel_scale(1) | wheel_gyro_bias(2) |
//          ext_delta_yaw(1) | ext_delta_xy(2)]
//
// Indices:
//   kLwoRotIdx     = 0   (3 elements: Lie algebra perturbation)
//   kLwoPosIdx     = 3   (3 elements)
//   kLwoVelIdx     = 6   (3 elements)
//   kLwoScaleIdx   = 9   (1 element: v_x^true = scale * v_x^enc)
//   kLwoBiasIdx    = 10  (2 elements: [b_omega_x, b_omega_z])
//   kLwoExtYawIdx  = 12  (1 element: extrinsic delta yaw)
//   kLwoExtXyIdx   = 13  (2 elements: extrinsic delta [x, y])

#ifndef TOF_SLAM_FRONTEND_W_ESTIMATOR_LWO_STATE_HPP_
#define TOF_SLAM_FRONTEND_W_ESTIMATOR_LWO_STATE_HPP_

#include <Eigen/Dense>
#include <iosfwd>

#include "tof_slam/common/lie/se3.hpp"

namespace tof_slam {
namespace lwo {

// ---------------------------------------------------------------------------
// Dimension constants
// ---------------------------------------------------------------------------
inline constexpr int kLwoStateDim   = 15;
inline constexpr int kLwoRotIdx     = 0;
inline constexpr int kLwoPosIdx     = 3;
inline constexpr int kLwoVelIdx     = 6;
inline constexpr int kLwoScaleIdx   = 9;
inline constexpr int kLwoBiasIdx    = 10;  // 2 elements: [b_omega_x, b_omega_z]
inline constexpr int kLwoExtYawIdx  = 12;  // 1 element: delta yaw for extrinsic
inline constexpr int kLwoExtXyIdx   = 13;  // 2 elements: delta [x, y] for extrinsic

using LwoStateVector    = Eigen::Matrix<float, kLwoStateDim, 1>;
using LwoStateCovariance = Eigen::Matrix<float, kLwoStateDim, kLwoStateDim>;

// ---------------------------------------------------------------------------
// LwoState
// ---------------------------------------------------------------------------
struct LwoState {
  Eigen::Matrix3f rotation    = Eigen::Matrix3f::Identity();
  Eigen::Vector3f position    = Eigen::Vector3f::Zero();
  Eigen::Vector3f velocity    = Eigen::Vector3f::Zero();

  /// Wheel encoder scale factor: v_x^true = wheel_scale * v_x^enc.
  /// Initialized to 1.0 (no scale error).
  float wheel_scale           = 1.0f;

  /// Wheel angular velocity biases: [b_omega_x, b_omega_z].
  /// b_omega_x corrects roll-rate measurement, b_omega_z corrects yaw-rate.
  Eigen::Vector2f wheel_gyro_bias = Eigen::Vector2f::Zero();

  /// Online extrinsic calibration: delta yaw correction applied to T_body_lidar.
  /// Units: radians.  Zero = no correction from initial extrinsic.
  float ext_delta_yaw = 0.0f;

  /// Online extrinsic calibration: delta XY translation applied to T_body_lidar.
  /// Units: meters.  Zero = no correction from initial extrinsic.
  Eigen::Vector2f ext_delta_xy = Eigen::Vector2f::Zero();

  LwoStateCovariance covariance = LwoStateCovariance::Identity() * 0.01f;

  // -- Manifold operators ---------------------------------------------------

  /// state + delta  (rotation via SO(3)::Exp, Euclidean rest via addition).
  LwoState operator+(const LwoStateVector& delta) const;

  /// state_this - state_other  (rotation via SO(3)::Log, rest via subtraction).
  LwoStateVector operator-(const LwoState& other) const;

  /// In-place increment.
  LwoState& operator+=(const LwoStateVector& delta);

  // -- Accessors ------------------------------------------------------------

  /// Build SE(3) pose from rotation and position.
  core::Se3 pose() const;

  /// Set rotation and position from an SE(3) pose.
  void set_pose(const core::Se3& T);

  /// Reset to identity state.
  void reset();
};

/// Human-readable output.
std::ostream& operator<<(std::ostream& os, const LwoState& state);

}  // namespace lwo
}  // namespace tof_slam

#endif  // TOF_SLAM_FRONTEND_W_ESTIMATOR_LWO_STATE_HPP_
