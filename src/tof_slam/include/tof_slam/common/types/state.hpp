// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// state.hpp — 18-dimensional LIO state with manifold operators.
//
// Layout: [rotation(3) | position(3) | velocity(3) |
//          gyro_bias(3) | acc_bias(3) | gravity(3)]

#ifndef TOF_SLAM_COMMON_TYPES_STATE_HPP_
#define TOF_SLAM_COMMON_TYPES_STATE_HPP_

#include <Eigen/Dense>
#include <iosfwd>

#include "tof_slam/common/lie/se3.hpp"

namespace tof_slam {
namespace core {

// ---------------------------------------------------------------------------
// Index constants
// ---------------------------------------------------------------------------
inline constexpr int kStateDim   = 18;
inline constexpr int kRotIdx     = 0;
inline constexpr int kPosIdx     = 3;
inline constexpr int kVelIdx     = 6;
inline constexpr int kGyrBiasIdx = 9;
inline constexpr int kAccBiasIdx = 12;
inline constexpr int kGravIdx    = 15;
inline constexpr float kGravityNorm = 9.81f;

using StateVector    = Eigen::Matrix<float, kStateDim, 1>;
// Covariance stored in float64 to prevent precision loss over long
// sequences. The 18x18 matrix has condition number ~1e8; float32 storage
// loses ~2-3 significant digits per store-load cycle for bias blocks.
using StateCovariance = Eigen::Matrix<double, kStateDim, kStateDim>;

// ---------------------------------------------------------------------------
// LioState
// ---------------------------------------------------------------------------
struct LioState {
  Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
  Eigen::Vector3f position   = Eigen::Vector3f::Zero();
  Eigen::Vector3f velocity   = Eigen::Vector3f::Zero();
  Eigen::Vector3f gyro_bias  = Eigen::Vector3f::Zero();
  Eigen::Vector3f acc_bias   = Eigen::Vector3f::Zero();
  Eigen::Vector3f gravity    = Eigen::Vector3f(0.0f, 0.0f, -kGravityNorm);

  StateCovariance covariance = StateCovariance::Identity() * 0.01;

  // -- Manifold operators ---------------------------------------------------

  /// state + delta  (rotation via SO(3)::Exp, rest via vector addition).
  LioState operator+(const StateVector& delta) const;

  /// state_this - state_other  (rotation via SO(3)::Log, rest via subtraction).
  StateVector operator-(const LioState& other) const;

  /// In-place increment.
  LioState& operator+=(const StateVector& delta);

  // -- Accessors ------------------------------------------------------------

  /// Build SE(3) pose from rotation and position.
  Se3 pose() const;

  /// Set rotation and position from an SE(3) pose.
  void set_pose(const Se3& T);

  /// Reset to default identity state.
  void reset();
};

/// Human-readable output.
std::ostream& operator<<(std::ostream& os, const LioState& state);

}  // namespace core
}  // namespace tof_slam

#endif  // TOF_SLAM_COMMON_TYPES_STATE_HPP_
