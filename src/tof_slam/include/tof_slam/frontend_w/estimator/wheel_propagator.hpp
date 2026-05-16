// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// wheel_propagator.hpp — Wheel-odometry based state propagation for LWO.
//
// Implements Euler integration of the 12-D LwoState using wheel encoder
// measurements.  Replaces the IMU-based imu_propagator for the no-IMU case.
//
// Body-frame velocity (nonholonomic ground robot):
//   v^b = [s * v_x^enc, 0, 0]^T
//   omega^b = [-b_omega_x, 0, omega_z^enc - b_omega_z]^T
//
// State propagation:
//   R(t+dt) = R(t) * Exp(omega^b * dt)
//   v(t+dt) = R(t) * v^b            (world-frame velocity update)
//   p(t+dt) = p(t) + v(t) * dt
//   s, b_w: random walk (unchanged in mean)
//
// Covariance: P(t+dt) = F_w * P * F_w^T + Q_w * dt

#ifndef TOF_SLAM_FRONTEND_W_ESTIMATOR_WHEEL_PROPAGATOR_HPP_
#define TOF_SLAM_FRONTEND_W_ESTIMATOR_WHEEL_PROPAGATOR_HPP_

#include <Eigen/Dense>

#include "tof_slam/frontend_w/estimator/lwo_state.hpp"

namespace tof_slam {
namespace lwo {

// ---------------------------------------------------------------------------
// WheelPropagatorConfig
// ---------------------------------------------------------------------------

struct WheelPropagatorConfig {
  // Process noise standard deviations (continuous-time, scaled by sqrt(dt))
  // Tuned to match LIO's effective P growth per LiDAR interval:
  //   LIO: 200Hz IMU, sigma=0.1 → P_growth ≈ 1e-3 per 0.1s
  //   LWO: 100Hz wheel → need sigma ≈ 0.05~0.1 for similar P_growth
  float sigma_rot   = 0.05f;    // rad/sqrt(s)  — rotation process noise
  float sigma_pos   = 0.01f;    // m/sqrt(s)    — position process noise
  float sigma_vel   = 0.05f;    // m/s/sqrt(s)  — velocity process noise
  float sigma_scale = 1e-4f;    // 1/sqrt(s)    — wheel scale random walk (tight)
  float sigma_bias  = 0.0f;     // rad/s/sqrt(s) — bias frozen

  // Lever arm: translation from body (wheel center) to LiDAR in body frame.
  // Used for lever arm-aware process noise scaling: during rotation, position
  // uncertainty increases proportionally to |lever_arm| × |ω|.
  // Default zero = no lever arm effect (backward compatible with magok_caffe).
  Eigen::Vector3f lever_arm = Eigen::Vector3f::Zero();

  // Online extrinsic calibration process noise.
  // Zero = extrinsic is constant (no random walk).
  float sigma_ext_yaw = 0.0f;  // Extrinsic yaw process noise (rad/sqrt(s))
  float sigma_ext_xy  = 0.0f;  // Extrinsic XY process noise (m/sqrt(s))
};

// ---------------------------------------------------------------------------
// WheelPropagator
// ---------------------------------------------------------------------------

/// Propagates a LwoState forward by one wheel odometry step.
class WheelPropagator {
 public:
  explicit WheelPropagator(const WheelPropagatorConfig& cfg = {});

  /// Propagate state forward by dt seconds given encoder readings.
  ///
  /// @param prior        Current LwoState.
  /// @param vx_enc       Forward wheel velocity from encoder (m/s), body frame.
  /// @param omega_z_enc  Yaw rate from encoder (rad/s), body frame.
  /// @param dt           Time step (seconds).  Clamped to (0, 1].
  /// @return             Propagated state with updated covariance.
  LwoState propagate(const LwoState& prior,
                     float vx_enc,
                     float omega_z_enc,
                     float dt) const;

  /// Build the 12×12 discrete-time transition matrix F_w.
  ///
  /// @param state        State at time t (used for R, v^b, scale, biases).
  /// @param vx_enc       Forward encoder velocity (m/s).
  /// @param omega_z_enc  Yaw encoder rate (rad/s).
  /// @param dt           Time step (seconds).
  LwoStateCovariance build_transition_matrix(const LwoState& state,
                                              float vx_enc,
                                              float omega_z_enc,
                                              float dt) const;

  /// Build the 12×12 process noise covariance Q_w (continuous-time diagonal).
  LwoStateCovariance process_noise() const;

 private:
  WheelPropagatorConfig cfg_;
};

}  // namespace lwo
}  // namespace tof_slam

#endif  // TOF_SLAM_FRONTEND_W_ESTIMATOR_WHEEL_PROPAGATOR_HPP_
