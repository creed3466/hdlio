// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// imu_propagator.hpp — Stateless IMU forward propagation.
//
// Implements Euler integration of the 18-D LIO state given a single IMU
// measurement and time step.  All logic extracted from Estimator::PropagateState.

#ifndef TOF_SLAM_FRONTEND_ESTIMATOR_IMU_PROPAGATOR_HPP_
#define TOF_SLAM_FRONTEND_ESTIMATOR_IMU_PROPAGATOR_HPP_

#include "tof_slam/common/types/imu_types.hpp"
#include "tof_slam/common/types/state.hpp"

namespace tof_slam {
namespace core {

/// Propagate `prior` state forward by `dt` seconds using `imu` measurement.
///
/// - Returns the prior unchanged if dt <= 0 or dt > 1.0 (safety guard).
/// - `process_noise` (Q) is the diagonal noise matrix, already scaled per
///   unit-time.  It is multiplied by dt internally.
///
/// This is a pure function — no side effects, no internal state.
LioState propagate_imu(const LioState& prior,
                       const ImuMeasurement& imu,
                       float dt,
                       const StateCovariance& process_noise);

/// Build the 18x18 state transition matrix F for linearised IMU dynamics.
/// Exposed for unit-test verification (finite-difference Jacobian check).
StateCovariance build_transition_matrix(const LioState& state,
                                        const ImuMeasurement& imu,
                                        float dt);

/// FAST-LIO2 style IMU propagation with F_w noise input matrix.
///
/// Key differences from propagate_imu():
///   1. Uses averaged IMU (caller provides pre-averaged measurement)
///   2. Covariance: P = F*P*F^T + dt²·F_w·Q_12·F_w^T
///      where Q_12 is 12x12 (ng, na, nbg, nba) and F_w maps noise to state
///   3. F_w couples acc noise through rotation: F_w[vel, na] = -R
///   4. F[rot,rot] uses Exp(-omega*dt) instead of I - [omega]_x*dt
///
/// `process_noise_12` is 12x12 diagonal: [ng(3), na(3), nbg(3), nba(3)]
LioState propagate_imu_fastlio(const LioState& prior,
                               const ImuMeasurement& imu,
                               float dt,
                               const Eigen::Matrix<double, 12, 12>& process_noise_12);


}  // namespace core
}  // namespace tof_slam

#endif  // TOF_SLAM_FRONTEND_ESTIMATOR_IMU_PROPAGATOR_HPP_
