// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "tof_slam/frontend/estimator/gravity_init.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <sstream>

namespace tof_slam {
namespace core {

GravityInitResult initialize_gravity(
    const std::vector<ImuMeasurement>& buffer,
    const Eigen::Vector3f& gravity_prior,
    int min_samples) {
  GravityInitResult result;

  // 1. Check minimum sample count.
  if (static_cast<int>(buffer.size()) < min_samples) {
    result.message = "Not enough IMU samples (need >= " +
                     std::to_string(min_samples) + ", got " +
                     std::to_string(buffer.size()) + ")";
    return result;
  }

  // 2. Compute mean acceleration and gyroscope.
  Eigen::Vector3f mean_acc = Eigen::Vector3f::Zero();
  Eigen::Vector3f mean_gyr = Eigen::Vector3f::Zero();
  for (const auto& imu : buffer) {
    mean_acc += imu.accel;
    mean_gyr += imu.gyro;
  }
  const float n = static_cast<float>(buffer.size());
  mean_acc /= n;
  mean_gyr /= n;

  // 3. Auto-detect g-units: if |mean_acc| < 2.0, sensor likely reports in
  //    g-units (MID-360 style) rather than m/s^2.
  const float acc_norm = mean_acc.norm();
  const float gravity_magnitude = gravity_prior.norm();

  if (acc_norm < 2.0f && acc_norm > 0.1f) {
    result.imu_acc_scale = gravity_magnitude / acc_norm;
    // Apply scale to mean_acc for initialization.
    mean_acc *= result.imu_acc_scale;
  }

  // 4. Check if accelerometer norm is reasonable.
  const float scaled_norm = mean_acc.norm();
  if (std::abs(scaled_norm - gravity_magnitude) > 1.5f) {
    result.message = "Accelerometer norm (" + std::to_string(scaled_norm) +
                     " m/s^2) too far from gravity (" +
                     std::to_string(gravity_magnitude) + ")";
    return result;
  }

  // 5. Compute variance for stationarity check.
  float acc_variance = 0.0f;
  for (const auto& imu : buffer) {
    Eigen::Vector3f a = imu.accel;
    if (result.imu_acc_scale > 1.01f) a *= result.imu_acc_scale;
    acc_variance += (a - mean_acc).squaredNorm();
  }
  acc_variance /= n;

  // Check gyro variance to detect rotation during init.
  float gyr_variance = 0.0f;
  for (const auto& imu : buffer) {
    gyr_variance += (imu.gyro - mean_gyr).squaredNorm();
  }
  gyr_variance /= n;

  // Determine stationarity.  Only use mean_gyr for bias init if robot
  // is roughly stationary; otherwise set biases to zero and let IEKF
  // estimate them online.
  const bool is_stationary = (gyr_variance < 0.01f && acc_variance < 1.0f);

  // Build initial state.
  LioState& state = result.initial_state;
  state.reset();

  // 6. Gravity alignment: rotate world frame so measured gravity matches prior.
  const Eigen::Vector3f gravity_measured =
      -mean_acc.normalized() * gravity_magnitude;
  state.gravity = gravity_measured;

  const Eigen::Quaternionf q_align = Eigen::Quaternionf::FromTwoVectors(
      state.gravity.normalized(), gravity_prior.normalized());
  const Eigen::Matrix3f R_align = q_align.toRotationMatrix();

  state.rotation = R_align;
  state.gravity = R_align * state.gravity;  // Should now ≈ gravity_prior.

  // 7. Biases.
  if (is_stationary) {
    // Stationary: estimate biases from init data.
    state.gyro_bias = mean_gyr;
    // acc_bias = mean_acc + R^T * g_world
    state.acc_bias =
        mean_acc + state.rotation.transpose() * gravity_prior;
  } else {
    // Moving: set biases to zero.  During motion, mean_gyr includes
    // actual rotation (not just bias), so it corrupts the estimate.
    state.gyro_bias.setZero();
    state.acc_bias.setZero();
  }

  state.position.setZero();
  state.velocity.setZero();

  // 8. Covariance.
  // When non-stationary, use larger initial uncertainty for biases and
  // rotation (compensate for poorer initialization).
  state.covariance = StateCovariance::Identity();
  if (is_stationary) {
    state.covariance.block<3, 3>(kRotIdx, kRotIdx)         *= 0.01f;
    state.covariance.block<3, 3>(kPosIdx, kPosIdx)         *= 1.0f;
    state.covariance.block<3, 3>(kVelIdx, kVelIdx)         *= 0.1f;
    state.covariance.block<3, 3>(kGyrBiasIdx, kGyrBiasIdx) *= 0.001f;
    state.covariance.block<3, 3>(kAccBiasIdx, kAccBiasIdx) *= 0.01f;
  } else {
    state.covariance.block<3, 3>(kRotIdx, kRotIdx)         *= 0.1f;
    state.covariance.block<3, 3>(kPosIdx, kPosIdx)         *= 1.0f;
    state.covariance.block<3, 3>(kVelIdx, kVelIdx)         *= 0.5f;
    state.covariance.block<3, 3>(kGyrBiasIdx, kGyrBiasIdx) *= 0.01f;
    state.covariance.block<3, 3>(kAccBiasIdx, kAccBiasIdx) *= 0.1f;
  }
  state.covariance.block<3, 3>(kGravIdx, kGravIdx)   *= 0.001f;

  result.success = true;

  // Record the last IMU timestamp so LioEstimator can seed last_imu_time_.
  result.last_imu_timestamp = buffer.back().timestamp;

  std::ostringstream oss;
  oss << "Gravity init OK: " << buffer.size() << " samples, "
      << "acc_var=" << acc_variance << ", "
      << "scale=" << result.imu_acc_scale;
  if (acc_variance > 0.5f) oss << " [WARNING: high acc variance, robot may be moving]";
  result.message = oss.str();

  return result;
}

}  // namespace core
}  // namespace tof_slam
