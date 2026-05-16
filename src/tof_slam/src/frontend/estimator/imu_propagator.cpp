// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "tof_slam/frontend/estimator/imu_propagator.hpp"

#include "tof_slam/common/lie/so3.hpp"

namespace tof_slam {
namespace core {

StateCovariance build_transition_matrix(const LioState& state,
                                        const ImuMeasurement& imu,
                                        float dt) {
  const Eigen::Matrix3f R = state.rotation;
  const Eigen::Vector3f omega = imu.gyro - state.gyro_bias;
  const Eigen::Vector3f acc = imu.accel - state.acc_bias;

  StateCovariance F = StateCovariance::Identity();

  const double dtd = static_cast<double>(dt);

  // Rotation block: dR/d(delta_rot) ≈ I - [omega]_x * dt
  F.block<3, 3>(kRotIdx, kRotIdx) =
      (Eigen::Matrix3f::Identity() - Hat(omega) * dt).cast<double>();
  // Rotation vs gyro bias:
  F.block<3, 3>(kRotIdx, kGyrBiasIdx) = (-R * dt).cast<double>();

  // Position block: dp/dv = I * dt
  F.block<3, 3>(kPosIdx, kVelIdx) = Eigen::Matrix3d::Identity() * dtd;

  // Velocity block (mid-point: evaluate at R_mid for consistency):
  const Eigen::Matrix3f R_mid = R * So3::Exp(omega * (0.5f * dt)).matrix();
  F.block<3, 3>(kVelIdx, kRotIdx) = (-R_mid * Hat(acc) * dt).cast<double>();
  F.block<3, 3>(kVelIdx, kAccBiasIdx) = (-R_mid * dt).cast<double>();
  F.block<3, 3>(kVelIdx, kGravIdx) = Eigen::Matrix3d::Identity() * dtd;

  return F;
}

LioState propagate_imu(const LioState& prior,
                       const ImuMeasurement& imu,
                       float dt,
                       const StateCovariance& process_noise) {
  // Safety guard.
  if (dt <= 0.0f || dt > 1.0f) return prior;

  LioState result = prior;

  // Bias-corrected measurements.
  const Eigen::Vector3f omega = imu.gyro - prior.gyro_bias;
  const Eigen::Vector3f acc = imu.accel - prior.acc_bias;

  const Eigen::Matrix3f R = prior.rotation;

  // --- Mid-point integration (reduces per-step error from O(dt²) to O(dt³)) ---
  // Evaluate rotation and acceleration at t + dt/2 for improved accuracy.
  const Eigen::Matrix3f R_mid = R * So3::Exp(omega * (0.5f * dt)).matrix();
  const Eigen::Vector3f acc_world_mid = R_mid * acc + prior.gravity;

  // Rotation:  R(t+dt) = R(t) * Exp(omega * dt)
  result.rotation = R * So3::Exp(omega * dt).matrix();

  // Velocity:  v(t+dt) = v(t) + acc_world_mid * dt
  result.velocity = prior.velocity + acc_world_mid * dt;

  // Position:  p(t+dt) = p(t) + v*dt + 0.5*a_mid*dt^2
  result.position =
      prior.position + prior.velocity * dt + 0.5f * acc_world_mid * dt * dt;

  // Biases and gravity: random walk (unchanged in mean).
  // (already copied from prior)

  // --- Covariance propagation ---
  // P(t+dt) = F * P * F^T + Q * dt
  //
  // Computed in float64 to prevent condition-number degradation over
  // hundreds of IMU propagation steps per LiDAR frame.  The F*P*F^T
  // product involves near-cancellation for the rotation block (small omega*dt
  // multiplied by small P_rot), and float32 truncation accumulates
  // systematically.  Cost: ~0.005ms per IMU step (negligible).
  // StateCovariance is float64 — no cast needed.
  const StateCovariance F = build_transition_matrix(prior, imu, dt);
  result.covariance = F * prior.covariance * F.transpose()
                    + process_noise * static_cast<double>(dt);

  return result;
}


// ---------------------------------------------------------------------------
// FAST-LIO2 style IMU propagation with F_w noise input matrix.
// ---------------------------------------------------------------------------

LioState propagate_imu_fastlio(const LioState& prior,
                               const ImuMeasurement& imu,
                               float dt,
                               const Eigen::Matrix<double, 12, 12>& Q12) {
  if (dt <= 0.0f || dt > 1.0f) return prior;

  LioState result = prior;

  // Bias-corrected measurements (caller already averaged if desired).
  const Eigen::Vector3f omega = imu.gyro - prior.gyro_bias;
  const Eigen::Vector3f acc = imu.accel - prior.acc_bias;

  const Eigen::Matrix3f R = prior.rotation;

  // --- State integration (Forward Euler with midpoint R, same as original) ---
  const Eigen::Matrix3f R_mid = R * So3::Exp(omega * (0.5f * dt)).matrix();
  const Eigen::Vector3f acc_world_mid = R_mid * acc + prior.gravity;

  result.rotation = R * So3::Exp(omega * dt).matrix();
  result.velocity = prior.velocity + acc_world_mid * dt;
  result.position =
      prior.position + prior.velocity * dt + 0.5f * acc_world_mid * dt * dt;

  // --- Covariance propagation (FAST-LIO2 style) ---
  //
  // F_x: 18x18 state transition Jacobian (with manifold correction).
  // Key difference: F[rot,rot] = Exp(-omega*dt) instead of I - [omega]_x*dt
  //
  const double dtd = static_cast<double>(dt);

  StateCovariance F = StateCovariance::Identity();

  // Rotation block: use Exp(-omega*dt) for manifold correction.
  const Eigen::Vector3f neg_omega_dt = -omega * dt;
  F.block<3, 3>(kRotIdx, kRotIdx) =
      So3::Exp(neg_omega_dt).matrix().cast<double>();
  F.block<3, 3>(kRotIdx, kGyrBiasIdx) = (-R * dt).cast<double>();

  // Position
  F.block<3, 3>(kPosIdx, kVelIdx) = Eigen::Matrix3d::Identity() * dtd;

  // Velocity (midpoint R)
  F.block<3, 3>(kVelIdx, kRotIdx) = (-R_mid * Hat(acc) * dt).cast<double>();
  F.block<3, 3>(kVelIdx, kAccBiasIdx) = (-R_mid * dt).cast<double>();
  F.block<3, 3>(kVelIdx, kGravIdx) = Eigen::Matrix3d::Identity() * dtd;

  // --- F_w: 18x12 noise input matrix ---
  //
  // Noise vector w = [ng(3), na(3), nbg(3), nba(3)]
  //
  //   F_w[rot,  ng]  = -I   (gyro noise -> rotation)
  //   F_w[vel,  na]  = -R   (acc noise -> velocity, rotated by R)
  //   F_w[bg,  nbg]  =  I   (gyro bias random walk)
  //   F_w[ba,  nba]  =  I   (acc bias random walk)
  //
  Eigen::Matrix<double, 18, 12> Fw = Eigen::Matrix<double, 18, 12>::Zero();
  Fw.block<3, 3>(kRotIdx, 0)     = -Eigen::Matrix3d::Identity();  // ng -> rot
  Fw.block<3, 3>(kVelIdx, 3)     = (-R_mid).cast<double>();        // na -> vel
  Fw.block<3, 3>(kGyrBiasIdx, 6) =  Eigen::Matrix3d::Identity();  // nbg -> bg
  Fw.block<3, 3>(kAccBiasIdx, 9) =  Eigen::Matrix3d::Identity();  // nba -> ba

  // P = F * P * F^T + dt² * F_w * Q12 * F_w^T
  //
  // The dt² factor comes from FAST-LIO2's discretization:
  //   discrete_noise = (dt * F_w) * Q * (dt * F_w)^T = dt² * F_w * Q * F_w^T
  //
  const Eigen::Matrix<double, 18, 12> dtFw = dtd * Fw;
  result.covariance = F * prior.covariance * F.transpose()
                    + dtFw * Q12 * dtFw.transpose();

  return result;
}

}  // namespace core
}  // namespace tof_slam
