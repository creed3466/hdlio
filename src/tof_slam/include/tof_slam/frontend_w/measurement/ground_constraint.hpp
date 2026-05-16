// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// ground_constraint.hpp — Ground-plane pseudo-measurement for LWO.
//
// Constrains roll, pitch, and z-position to near-zero for a flat-floor
// ground robot.  Implemented as a pseudo-measurement with configurable noise.
//
// Residual (3×1):  r_gc = [phi_roll, phi_pitch, p_z]^T
//
// Jacobian H_gc (3×12) — state layout [phi(0:3), p(3:6), v(6:9), s(9), bw(10:12)]:
//   row 0: [1, 0, 0, 0,0,0, 0,0,0, 0, 0,0]  <- d(phi_roll)/d(phi)
//   row 1: [0, 1, 0, 0,0,0, 0,0,0, 0, 0,0]  <- d(phi_pitch)/d(phi)
//   row 2: [0, 0, 0, 0,0,1, 0,0,0, 0, 0,0]  <- d(p_z)/d(p)

#ifndef TOF_SLAM_FRONTEND_W_MEASUREMENT_GROUND_CONSTRAINT_HPP_
#define TOF_SLAM_FRONTEND_W_MEASUREMENT_GROUND_CONSTRAINT_HPP_

#include <Eigen/Dense>

#include "tof_slam/frontend_w/estimator/lwo_state.hpp"

namespace tof_slam {
namespace lwo {

// ---------------------------------------------------------------------------
// GroundConstraintConfig
// ---------------------------------------------------------------------------

struct GroundConstraintConfig {
  float noise_roll_pitch = 0.01f;   // rad  — roll/pitch constraint noise std
  float noise_z          = 0.001f;  // m    — z-position constraint noise std
};

// ---------------------------------------------------------------------------
// MeasurementResult — generic container for (H, residual, noise_cov)
// ---------------------------------------------------------------------------

template <int MeasDim>
struct MeasurementResult {
  Eigen::Matrix<float, MeasDim, kLwoStateDim> H;
  Eigen::Matrix<float, MeasDim, 1>            residual;
  Eigen::Matrix<float, MeasDim, MeasDim>      noise_cov;
};

using GroundConstraintResult = MeasurementResult<3>;

// ---------------------------------------------------------------------------
// GroundConstraint
// ---------------------------------------------------------------------------

class GroundConstraint {
 public:
  explicit GroundConstraint(const GroundConstraintConfig& cfg = {});

  /// Compute H, residual, and noise covariance for the current state.
  GroundConstraintResult compute(const LwoState& state) const;

  /// Build the 3×12 Jacobian H_gc for the given state.
  /// (Mostly constant but returned per-call for API consistency.)
  Eigen::Matrix<float, 3, kLwoStateDim> jacobian() const;

  /// Measurement noise covariance R_gc (3×3).
  Eigen::Matrix3f noise() const;

 private:
  GroundConstraintConfig cfg_;
};

}  // namespace lwo
}  // namespace tof_slam

#endif  // TOF_SLAM_FRONTEND_W_MEASUREMENT_GROUND_CONSTRAINT_HPP_
