// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "tof_slam/frontend_w/estimator/wheel_propagator.hpp"

#include <cmath>

#include "tof_slam/common/lie/so3.hpp"

namespace tof_slam {
namespace lwo {

WheelPropagator::WheelPropagator(const WheelPropagatorConfig& cfg)
    : cfg_(cfg) {}

// ---------------------------------------------------------------------------
// build_transition_matrix
// ---------------------------------------------------------------------------

LwoStateCovariance WheelPropagator::build_transition_matrix(
    const LwoState& state,
    float vx_enc,
    float omega_z_enc,
    float dt) const {

  const Eigen::Matrix3f& R = state.rotation;
  const float s = state.wheel_scale;
  const float b_ox = state.wheel_gyro_bias(0);
  const float b_oz = state.wheel_gyro_bias(1);

  // Body-frame angular velocity (with bias correction).
  const Eigen::Vector3f omega_b(-b_ox, 0.0f, omega_z_enc - b_oz);

  // Body-frame velocity vector (nonholonomic: only v_x non-zero).
  const Eigen::Vector3f v_b(s * vx_enc, 0.0f, 0.0f);

  LwoStateCovariance F = LwoStateCovariance::Identity();

  // ---- Row block: d(phi) / d(...) ----------------------------------------

  // d(phi)/d(phi): SO(3) first-order: I - [omega^b]_x * dt
  F.block<3, 3>(kLwoRotIdx, kLwoRotIdx) =
      Eigen::Matrix3f::Identity() - core::Hat(omega_b) * dt;

  // d(phi_body)/d(b_omega_x) = -e1 * dt  (col 10)
  // operator- returns body-frame delta via Log(R_other^T * R_this),
  // so the Jacobian must be expressed in body frame: delta_phi_body = -e1 * dt * db_ox
  F(kLwoRotIdx + 0, kLwoBiasIdx)     = -dt;  // e1 component
  // d(phi_body)/d(b_omega_z) = -e3 * dt  (col 11)
  F(kLwoRotIdx + 2, kLwoBiasIdx + 1) = -dt;  // e3 component

  // ---- Row block: d(p) / d(...) ------------------------------------------
  // Position uses trapezoidal (midpoint) integration:
  //   p(t+dt) = p(t) + 0.5*(v_old + v_new)*dt
  // where v_new = R*v^b depends on rotation and scale.

  // d(p)/d(phi): half the velocity rotation dependence (midpoint)
  //   v_new = R*v^b, d(R*v^b)/d(delta_phi) = -R*[v^b]_x
  F.block<3, 3>(kLwoPosIdx, kLwoRotIdx) = -0.5f * dt * R * core::Hat(v_b);

  // d(p)/d(v_old) = 0.5*I*dt  (midpoint: old velocity contributes half)
  F.block<3, 3>(kLwoPosIdx, kLwoVelIdx) = Eigen::Matrix3f::Identity() * 0.5f * dt;

  // d(p)/d(s) = 0.5*dt * R*e1*vx_enc  (v_new = R*[s*vx, 0, 0])
  F.block<3, 1>(kLwoPosIdx, kLwoScaleIdx) = 0.5f * dt * R.col(0) * vx_enc;

  // ---- Row block: d(v) / d(...) ------------------------------------------
  // Velocity is directly assigned: v(t+dt) = R(t)*v^b (not accumulated).
  // Therefore F[vel,vel] = 0 (velocity does NOT carry forward from prior).

  // d(v)/d(vel): 0 — velocity is directly assigned, not accumulated.
  F.block<3, 3>(kLwoVelIdx, kLwoVelIdx) = Eigen::Matrix3f::Zero();

  // d(v)/d(phi): world-frame velocity v = R*v^b (direct assignment)
  //   d(R*v^b)/d(delta_phi) = -R * [v^b]_x  (no dt — v is directly assigned)
  F.block<3, 3>(kLwoVelIdx, kLwoRotIdx) = -R * core::Hat(v_b);

  // d(v)/d(s) = R * e1 * v_x^enc  (no dt — direct assignment)
  F.block<3, 1>(kLwoVelIdx, kLwoScaleIdx) = R.col(0) * vx_enc;

  // d(v)/d(b_w) = 0  (nonholonomic: v_y^b = 0, bias does not affect world vel
  // directly — already zero from Identity initialization)

  // Scale and bias rows: identity (random walk, already set by Identity()).

  return F;
}

// ---------------------------------------------------------------------------
// process_noise
// ---------------------------------------------------------------------------

LwoStateCovariance WheelPropagator::process_noise() const {
  LwoStateCovariance Q = LwoStateCovariance::Zero();

  const float s2_rot   = cfg_.sigma_rot   * cfg_.sigma_rot;
  const float s2_pos   = cfg_.sigma_pos   * cfg_.sigma_pos;
  const float s2_vel   = cfg_.sigma_vel   * cfg_.sigma_vel;
  const float s2_scale = cfg_.sigma_scale * cfg_.sigma_scale;
  const float s2_bias  = cfg_.sigma_bias  * cfg_.sigma_bias;

  const float s2_ext_yaw = cfg_.sigma_ext_yaw * cfg_.sigma_ext_yaw;
  const float s2_ext_xy  = cfg_.sigma_ext_xy  * cfg_.sigma_ext_xy;

  Q.block<3, 3>(kLwoRotIdx,   kLwoRotIdx)   = Eigen::Matrix3f::Identity() * s2_rot;
  Q.block<3, 3>(kLwoPosIdx,   kLwoPosIdx)   = Eigen::Matrix3f::Identity() * s2_pos;
  Q.block<3, 3>(kLwoVelIdx,   kLwoVelIdx)   = Eigen::Matrix3f::Identity() * s2_vel;
  Q(kLwoScaleIdx, kLwoScaleIdx)              = s2_scale;
  Q.block<2, 2>(kLwoBiasIdx,  kLwoBiasIdx)  = Eigen::Matrix2f::Identity() * s2_bias;
  // Extrinsic: constant model, zero process noise by default.
  Q(kLwoExtYawIdx, kLwoExtYawIdx)                = s2_ext_yaw;
  Q.block<2, 2>(kLwoExtXyIdx, kLwoExtXyIdx) = Eigen::Matrix2f::Identity() * s2_ext_xy;

  return Q;
}

// ---------------------------------------------------------------------------
// propagate
// ---------------------------------------------------------------------------

LwoState WheelPropagator::propagate(const LwoState& prior,
                                     float vx_enc,
                                     float omega_z_enc,
                                     float dt) const {
  if (dt <= 0.0f || dt > 1.0f) return prior;

  LwoState result = prior;

  const Eigen::Matrix3f& R = prior.rotation;
  const float s    = prior.wheel_scale;
  const float b_ox = prior.wheel_gyro_bias(0);
  const float b_oz = prior.wheel_gyro_bias(1);

  // Bias-corrected body-frame kinematics.
  // Note: omega_z_enc is NOT scaled by wheel_scale because the robot's
  // odom yaw rate comes from an onboard gyroscope, not differential encoders.
  const Eigen::Vector3f omega_b(-b_ox, 0.0f, omega_z_enc - b_oz);
  const Eigen::Vector3f v_b(s * vx_enc, 0.0f, 0.0f);

  // Save prior velocity for trapezoidal (midpoint) position integration.
  const Eigen::Vector3f v_old = prior.velocity;

  // --- Rotation: R(t+dt) = R(t) * Exp(omega^b * dt) -----------------------
  result.rotation = R * core::So3::Exp(omega_b * dt).matrix();

  // --- World-frame velocity: v(t+dt) = R(t) * v^b -------------------------
  // Uses prior (old) rotation for consistency with build_transition_matrix.
  result.velocity = R * v_b;

  // --- Position: trapezoidal integration p(t+dt) = p(t) + 0.5*(v_old+v_new)*dt
  result.position = prior.position + 0.5f * (v_old + result.velocity) * dt;

  // --- Scale and bias: random walk (unchanged in mean) ----------------------
  // (already copied from prior)

  // --- Covariance propagation: P = F * P * F^T + Q * dt --------------------
  const LwoStateCovariance F = build_transition_matrix(prior, vx_enc, omega_z_enc, dt);
  LwoStateCovariance Q = process_noise();

  // Velocity-dependent process noise scaling (I2EKF-LO inspired):
  //   Higher rotation uncertainty during turning
  //   Higher position uncertainty at higher speed
  //   Reduced uncertainty when stationary
  const float abs_vx = std::abs(s * vx_enc);
  const float abs_wz = std::abs(omega_z_enc - b_oz);

  // Rotation: scale with angular velocity (turning → more heading uncertainty)
  const float rot_scale = 1.0f + 3.0f * abs_wz;  // 1.0 at rest, ~2.5 at 0.5 rad/s
  for (int i = 0; i < 3; ++i)
    Q(kLwoRotIdx + i, kLwoRotIdx + i) *= rot_scale;

  // Position: scale with linear velocity
  const float pos_scale = 1.0f + 2.0f * abs_vx;  // 1.0 at rest, ~2.6 at 0.8 m/s
  for (int i = 0; i < 3; ++i)
    Q(kLwoPosIdx + i, kLwoPosIdx + i) *= pos_scale;

  // Velocity: scale with linear velocity
  const float vel_scale = 1.0f + 2.0f * abs_vx;
  for (int i = 0; i < 3; ++i)
    Q(kLwoVelIdx + i, kLwoVelIdx + i) *= vel_scale;

  // When ext calibration process noise is zero (default), use 12x12 block
  // propagation to preserve exact numerical backward compatibility with the
  // original 12-DOF code.  The ext covariance block is identity-propagated
  // (constant model), so we just preserve it unchanged.
  if (cfg_.sigma_ext_yaw == 0.0f && cfg_.sigma_ext_xy == 0.0f) {
    constexpr int kBaseDim = 12;
    const Eigen::Matrix<float, kBaseDim, kBaseDim> F12 =
        F.topLeftCorner<kBaseDim, kBaseDim>();
    const Eigen::Matrix<float, kBaseDim, kBaseDim> P12 =
        prior.covariance.topLeftCorner<kBaseDim, kBaseDim>();
    const Eigen::Matrix<float, kBaseDim, kBaseDim> Q12 =
        Q.topLeftCorner<kBaseDim, kBaseDim>();
    result.covariance = prior.covariance;  // preserve ext block as-is
    result.covariance.topLeftCorner<kBaseDim, kBaseDim>() =
        F12 * P12 * F12.transpose() + Q12 * dt;
    // Zero out cross-correlations (ext is decoupled when not estimated).
    result.covariance.topRightCorner<kBaseDim, kLwoStateDim - kBaseDim>().setZero();
    result.covariance.bottomLeftCorner<kLwoStateDim - kBaseDim, kBaseDim>().setZero();
  } else {
    result.covariance = F * prior.covariance * F.transpose() + Q * dt;
  }

  return result;
}

}  // namespace lwo
}  // namespace tof_slam
