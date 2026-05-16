// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "tof_slam/frontend/estimator/imu_preintegration.hpp"

#include "tof_slam/common/lie/so3.hpp"
#include "tof_slam/common/lie/se3.hpp"

namespace tof_slam {
namespace core {

ImuPreintegration preintegrate_imu(
    const std::vector<ImuMeasurement>& imu_buffer,
    const Eigen::Vector3f& bg,
    const Eigen::Vector3f& ba,
    float gyro_noise_density,
    float acc_noise_density) {
  ImuPreintegration result;
  result.bg_lin = bg;
  result.ba_lin = ba;

  if (imu_buffer.size() < 2) {
    return result;
  }

  // Noise covariance for a single IMU measurement (continuous-time).
  // Discrete-time: Q_d = Q_c / dt  → scaled by 1/dt in the loop below.
  //
  // 6×6: [gyro_noise(3), acc_noise(3)]
  Eigen::Matrix<double, 6, 6> Q_imu = Eigen::Matrix<double, 6, 6>::Zero();
  const double gn2 = static_cast<double>(gyro_noise_density * gyro_noise_density);
  const double an2 = static_cast<double>(acc_noise_density * acc_noise_density);
  Q_imu.diagonal().head<3>().setConstant(gn2);
  Q_imu.diagonal().tail<3>().setConstant(an2);

  // Running state in preintegration frame.
  Eigen::Matrix3f dR = Eigen::Matrix3f::Identity();
  Eigen::Vector3f dv = Eigen::Vector3f::Zero();
  Eigen::Vector3f dp = Eigen::Vector3f::Zero();

  // Covariance (9×9) and bias Jacobians (9×6 split into individual 3×3).
  Eigen::Matrix<double, 9, 9> cov = Eigen::Matrix<double, 9, 9>::Zero();

  // Bias Jacobians accumulated per step.
  Eigen::Matrix3f J_bg_r = Eigen::Matrix3f::Zero();
  Eigen::Matrix3f J_bg_v = Eigen::Matrix3f::Zero();
  Eigen::Matrix3f J_ba_v = Eigen::Matrix3f::Zero();
  Eigen::Matrix3f J_bg_p = Eigen::Matrix3f::Zero();
  Eigen::Matrix3f J_ba_p = Eigen::Matrix3f::Zero();

  double total_dt = 0.0;

  for (size_t i = 1; i < imu_buffer.size(); ++i) {
    const double dt = imu_buffer[i].timestamp - imu_buffer[i - 1].timestamp;
    if (dt <= 0.0 || dt > 1.0) continue;

    const float dtf = static_cast<float>(dt);
    total_dt += dt;

    // Midpoint of two consecutive IMU measurements (bias-corrected).
    const Eigen::Vector3f omega_prev = imu_buffer[i - 1].gyro - bg;
    const Eigen::Vector3f omega_curr = imu_buffer[i].gyro - bg;
    const Eigen::Vector3f omega_mid = 0.5f * (omega_prev + omega_curr);

    const Eigen::Vector3f acc_prev = imu_buffer[i - 1].accel - ba;
    const Eigen::Vector3f acc_curr = imu_buffer[i].accel - ba;
    const Eigen::Vector3f acc_mid = 0.5f * (acc_prev + acc_curr);

    // --- State integration (midpoint, matching propagate_imu) ---
    // Rotation at midpoint for acceleration transformation.
    const Eigen::Matrix3f dR_half = dR * So3::Exp(omega_mid * (0.5f * dtf)).matrix();
    const Eigen::Vector3f acc_in_preint = dR_half * acc_mid;

    // Update position before velocity (uses current velocity).
    dp += dv * dtf + 0.5f * acc_in_preint * dtf * dtf;
    dv += acc_in_preint * dtf;

    // Rotation: dR_{k+1} = dR_k * Exp(omega_mid * dt)
    const Eigen::Matrix3f dR_step = So3::Exp(omega_mid * dtf).matrix();
    const Eigen::Matrix3f dR_new = dR * dR_step;

    // --- Covariance propagation ---
    // 9×9 transition matrix A and 9×6 noise input matrix B.
    //
    // State ordering: [rot(3), vel(3), pos(3)]
    //
    // A = | Exp(-omega*dt)     0            0        |
    //     | -dR*[acc]_x*dt     I            0        |
    //     | -0.5*dR*[acc]_x*dt² dt*I        I        |
    //
    // B = | -I*dt              0            |
    //     | 0                  -dR*dt       |
    //     | 0                  -0.5*dR*dt²  |
    //
    const double dtd = dt;
    const Eigen::Matrix3d dR_d = dR.cast<double>();
    const Eigen::Matrix3d dR_step_T_d = dR_step.transpose().cast<double>();
    const Eigen::Matrix3d acc_hat_d = Hat(acc_mid).cast<double>();

    Eigen::Matrix<double, 9, 9> A = Eigen::Matrix<double, 9, 9>::Identity();
    // rot-rot: Exp(-omega*dt)^T ≈ dR_step^T
    A.block<3, 3>(0, 0) = dR_step_T_d;
    // vel-rot: -dR * [acc]_x * dt
    A.block<3, 3>(3, 0) = -dR_d * acc_hat_d * dtd;
    // pos-rot: -0.5 * dR * [acc]_x * dt²
    A.block<3, 3>(6, 0) = -0.5 * dR_d * acc_hat_d * dtd * dtd;
    // pos-vel: I * dt
    A.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity() * dtd;

    Eigen::Matrix<double, 9, 6> B = Eigen::Matrix<double, 9, 6>::Zero();
    // rot <- gyro noise: -I * dt
    B.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity() * dtd;
    // vel <- acc noise: -dR * dt
    B.block<3, 3>(3, 3) = -dR_d * dtd;
    // pos <- acc noise: -0.5 * dR * dt²
    B.block<3, 3>(6, 3) = -0.5 * dR_d * dtd * dtd;

    // P = A * P * A^T + B * Q_c * B^T / dt
    //
    // Q_imu is continuous-time noise density squared: σ² [rad²/s for gyro,
    // m²/s³ for accel].  White noise discretization gives n_d ~ N(0, Q_c/dt).
    // Since B already includes dt factors (B maps discrete noise to state
    // change: x += B * n_d), the covariance contribution is:
    //   B * (Q_c / dt) * B^T  =  B * Q_c * B^T / dt
    //
    // Verification (rotation block, 400Hz):
    //   B_rot = -I*dt → (-I*dt)*σ_g²*(-I*dt)^T / dt = σ_g² * dt  ✓
    cov = A * cov * A.transpose() + B * Q_imu * B.transpose() / dtd;

    // --- Bias Jacobians ---
    // J_bg_r: d(delta_R) / d(bg)
    //   J_bg_r_{k+1} = dR_step^T * J_bg_r_k - I * dt
    //   (simplified: ignoring right Jacobian for small dt)
    const Eigen::Matrix3f dR_step_T = dR_step.transpose();
    J_bg_r = dR_step_T * J_bg_r - Eigen::Matrix3f::Identity() * dtf;

    // J_bg_v: d(delta_v) / d(bg) = J_bg_v - dR * [acc]_x * J_bg_r * dt
    J_bg_v = J_bg_v - dR * Hat(acc_mid) * J_bg_r * dtf;

    // J_ba_v: d(delta_v) / d(ba) = J_ba_v - dR * dt
    J_ba_v = J_ba_v - dR * dtf;

    // J_bg_p: d(delta_p) / d(bg) = J_bg_p + J_bg_v * dt
    //         - 0.5 * dR * [acc]_x * J_bg_r * dt²
    J_bg_p = J_bg_p + J_bg_v * dtf
             - 0.5f * dR * Hat(acc_mid) * J_bg_r * dtf * dtf;

    // J_ba_p: d(delta_p) / d(ba) = J_ba_p + J_ba_v * dt - 0.5 * dR * dt²
    J_ba_p = J_ba_p + J_ba_v * dtf - 0.5f * dR * dtf * dtf;

    // Advance rotation.
    dR = dR_new;
    ++result.num_measurements;
  }

  result.delta_R = dR;
  result.delta_v = dv;
  result.delta_p = dp;
  result.delta_t = total_dt;
  result.covariance = cov;
  result.J_bg_rot = J_bg_r;
  result.J_bg_vel = J_bg_v;
  result.J_ba_vel = J_ba_v;
  result.J_bg_pos = J_bg_p;
  result.J_ba_pos = J_ba_p;

  return result;
}

}  // namespace core
}  // namespace tof_slam
