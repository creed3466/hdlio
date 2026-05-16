// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// wheel_measurement.hpp — Wheel velocity pseudo-measurement for LWO.
//
// Constrains body-frame forward velocity and yaw rate using encoder readings.
//
// Measurement:    z_wv = [v_x^enc, omega_z^enc]^T
// Predicted:      z_hat = [(1/s) * e1^T * R^T * v,  omega_z^b + b_omega_z]^T
//
// Residual (2×1): r_wv = z_wv - z_hat
//   r[0] = v_x^enc  - (1/s) * e1^T * R^T * v
//   r[1] = omega_z^enc - omega_z^b - b_omega_z
//
// Jacobian H_wv = dr/dx (2×12):
//   row 0: [(1/s)*e1^T*[R^T*v]×,  0_{1×3},  -(1/s)*e1^T*R^T,  vx_body/s^2,  0, 0]
//   row 1: [0_{1×3},               0_{1×3},   0_{1×3},          0,           0, -1]

#ifndef TOF_SLAM_FRONTEND_W_MEASUREMENT_WHEEL_MEASUREMENT_HPP_
#define TOF_SLAM_FRONTEND_W_MEASUREMENT_WHEEL_MEASUREMENT_HPP_

#include <Eigen/Dense>

#include "tof_slam/frontend_w/estimator/lwo_state.hpp"
#include "tof_slam/frontend_w/measurement/ground_constraint.hpp"  // MeasurementResult

namespace tof_slam {
namespace lwo {

// ---------------------------------------------------------------------------
// WheelMeasurementConfig
// ---------------------------------------------------------------------------

struct WheelMeasurementConfig {
  float noise_vx      = 0.05f;  // m/s  — forward velocity noise std
  float noise_omega_z = 0.05f;  // rad/s — yaw rate noise std
};

using WheelMeasurementResult = MeasurementResult<2>;

// ---------------------------------------------------------------------------
// WheelMeasurement
// ---------------------------------------------------------------------------

class WheelMeasurement {
 public:
  explicit WheelMeasurement(const WheelMeasurementConfig& cfg = {});

  /// Compute H, residual, and noise covariance.
  ///
  /// @param state        Current LwoState (R, v, scale, biases).
  /// @param vx_enc       Forward velocity from encoder (m/s), body frame.
  /// @param omega_z_enc  Yaw rate from encoder (rad/s), body frame.
  /// @param omega_z_b    Instantaneous body yaw rate (rad/s) used in row-1
  ///                     residual (typically omega_z^enc - b_omega_z).
  WheelMeasurementResult compute(const LwoState& state,
                                  float vx_enc,
                                  float omega_z_enc,
                                  float omega_z_b) const;

  /// Build the 2×12 Jacobian H_wv.
  Eigen::Matrix<float, 2, kLwoStateDim> jacobian(const LwoState& state) const;

  /// Measurement noise covariance R_wv (2×2).
  Eigen::Matrix2f noise() const;

 private:
  WheelMeasurementConfig cfg_;
};

}  // namespace lwo
}  // namespace tof_slam

#endif  // TOF_SLAM_FRONTEND_W_MEASUREMENT_WHEEL_MEASUREMENT_HPP_
