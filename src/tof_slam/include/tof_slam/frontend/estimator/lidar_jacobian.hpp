// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// lidar_jacobian.hpp — Point-to-plane Jacobian for the 18-D LIO state.
//
// Given a set of point-to-plane correspondences, computes the linearised
// measurement Jacobian H (N×18) and residual vector (N) used by the
// information-form Kalman update in the IEKF outer loop.
//
// Math summary (per row i):
//   p_imu   = R_il * p_lidar + t_il          (LiDAR → body/IMU)
//   p_world = R_wb * p_imu + t_wb            (body → world)
//   r_i     = -(n^T * p_world + plane_d)     (signed plane distance)
//
//   H[i, 0:3] = (R_wb^T * n)^T * [p_imu]×   (rotation Jacobian, right-perturbation)
//   H[i, 3:6] = -n^T                         (position Jacobian)
//   H[i, 6:18]= 0                            (vel/bias/gravity unobserved)

#ifndef TOF_SLAM_FRONTEND_ESTIMATOR_LIDAR_JACOBIAN_HPP_
#define TOF_SLAM_FRONTEND_ESTIMATOR_LIDAR_JACOBIAN_HPP_

#include <Eigen/Dense>
#include <vector>

#include "tof_slam/frontend/estimator/correspondence.hpp"
#include "tof_slam/common/lie/se3.hpp"
#include "tof_slam/common/types/state.hpp"

namespace tof_slam {
namespace core {

/// Compute point-to-plane Jacobians and residuals for the 18-D LIO state.
///
/// @param state          Current LIO state (rotation R_wb, position t_wb).
/// @param T_body_lidar   Extrinsic: rigid transform from LiDAR frame to body
///                       (IMU) frame.  Applied as:
///                         p_body = R_il * p_lidar + t_il
/// @param corrs          Point-to-plane correspondences (world-frame normals).
/// @param H              Output Jacobian matrix, resized to [N x 18].
/// @param residuals      Output residual vector, resized to [N].
void compute_lidar_jacobians(
    const LioState& state,
    const Se3& T_body_lidar,
    const std::vector<Correspondence>& corrs,
    Eigen::MatrixXf& H,
    Eigen::VectorXf& residuals);

/// Recompute only residuals at the updated state, reusing a cached Jacobian H.
///
/// This is an optimization for the IEKF inner loop: correspondences are fixed
/// within the inner loop, so H changes only through the rotation update
/// (a second-order effect when dx is small).  Recomputing residuals alone is
/// ~4x cheaper than full Jacobian+residual computation.
///
/// @param state          Current LIO state (rotation R_wb, position t_wb).
/// @param T_body_lidar   Extrinsic: LiDAR → body/IMU frame.
/// @param corrs          Same correspondences used for the cached H.
/// @param residuals      Output residual vector, resized to [N].
void recompute_residuals_only(
    const LioState& state,
    const Se3& T_body_lidar,
    const std::vector<Correspondence>& corrs,
    Eigen::VectorXf& residuals);

/// Optimized Jacobian: outputs Nx6 matrix (rot+pos only), uses pre-computed
/// p_imu to avoid redundant extrinsic transforms, and direct cross product.
///
/// Produces numerically identical results to compute_lidar_jacobians() but:
///   1. H6 is Nx6 (no zero columns, no setZero overhead)
///   2. p_imu_cache avoids recomputing R_il * p_lidar + t_il each iteration
///   3. Uses p_imu.cross(C) instead of Hat(p_imu) * C
///
/// @param state       Current LIO state (rotation R_wb, position t_wb).
/// @param p_imu_cache Pre-computed p_imu = R_il * p_lidar + t_il for each corr.
/// @param corrs       Point-to-plane correspondences.
/// @param H6          Output Jacobian matrix, resized to [N x 6].
/// @param residuals   Output residual vector, resized to [N].
void compute_lidar_jacobians_fast(
    const LioState& state,
    const std::vector<Eigen::Vector3f>& p_imu_cache,
    const std::vector<Correspondence>& corrs,
    Eigen::MatrixXf& H6,
    Eigen::VectorXf& residuals);

/// Residual-only recomputation using pre-computed p_imu cache.
///
/// @param state       Current LIO state.
/// @param p_imu_cache Pre-computed p_imu for each correspondence.
/// @param corrs       Same correspondences used for the cached H.
/// @param residuals   Output residual vector, resized to [N].
void recompute_residuals_fast(
    const LioState& state,
    const std::vector<Eigen::Vector3f>& p_imu_cache,
    const std::vector<Correspondence>& corrs,
    Eigen::VectorXf& residuals);

}  // namespace core
}  // namespace tof_slam

#endif  // TOF_SLAM_FRONTEND_ESTIMATOR_LIDAR_JACOBIAN_HPP_
