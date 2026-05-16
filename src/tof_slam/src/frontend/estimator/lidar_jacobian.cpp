// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// lidar_jacobian.cpp — Point-to-plane Jacobian for the 18-D LIO state.

#include "tof_slam/frontend/estimator/lidar_jacobian.hpp"

#include <omp.h>

#include "tof_slam/common/lie/so3.hpp"

namespace tof_slam {
namespace core {

namespace {
// Limit OpenMP threads to avoid oversubscription with Eigen's internal BLAS.
struct OmpThreadInit {
  OmpThreadInit() {
    const int max_threads = std::min(omp_get_max_threads(), 8);
    omp_set_num_threads(max_threads);
  }
} g_omp_init;
}  // namespace

void compute_lidar_jacobians(
    const LioState& state,
    const Se3& T_body_lidar,
    const std::vector<Correspondence>& corrs,
    Eigen::MatrixXf& H,
    Eigen::VectorXf& residuals) {
  const int n = static_cast<int>(corrs.size());

  // Always resize so callers see consistent shapes even when n == 0.
  H.resize(n, kStateDim);
  residuals.resize(n);

  if (n == 0) {
    return;
  }

  H.setZero();
  residuals.setZero();

  // Extract extrinsic components (LiDAR → body/IMU).
  const Eigen::Matrix3f R_il = T_body_lidar.rotation().matrix();
  const Eigen::Vector3f t_il = T_body_lidar.translation();

  // Extract body → world pose from current state.
  const Eigen::Matrix3f R_wb = state.rotation;
  const Eigen::Vector3f t_wb = state.position;

  for (int i = 0; i < n; ++i) {
    const Eigen::Vector3f& p_lidar = corrs[i].p_lidar;
    const Eigen::Vector3f& normal  = corrs[i].normal;
    const float            plane_d = corrs[i].plane_d;

    // --- Transform chain --------------------------------------------------
    // LiDAR frame → body/IMU frame
    const Eigen::Vector3f p_imu = R_il * p_lidar + t_il;

    // body/IMU frame → world frame
    const Eigen::Vector3f p_world = R_wb * p_imu + t_wb;

    // --- Residual ---------------------------------------------------------
    // Signed point-to-plane distance: n^T * p_world - plane_d
    // where plane_d = n^T * centroid.  Negated to match the Jacobian sign
    // convention (H_pos = -n^T).
    residuals(i) = -(normal.dot(p_world) - plane_d);

    // --- Jacobian ---------------------------------------------------------
    // Derivation (right-perturbation convention, matching LioState::operator+):
    //   R_perturbed = R_wb * Exp(delta_phi)
    //   p_world_perturbed = R_perturbed * p_imu + t_wb
    //   r_perturbed = -(n^T * p_world_perturbed + d)
    //
    // First-order expansion (Hat(delta) * v = delta × v = -Hat(v) * delta):
    //   dr/d_delta_phi = n^T * R_wb * Hat(p_imu)
    //                  = (R_wb^T * n)^T * Hat(p_imu)
    //                  = C^T * Hat(p_imu)
    //   Note: C^T * Hat(p_imu) = -(Hat(p_imu) * C)^T
    //
    //   dr/d_t_wb = -n^T

    const Eigen::Vector3f C = R_wb.transpose() * normal;

    // Rotation block (cols kRotIdx … kRotIdx+2):
    //   Row = C^T * Hat(p_imu)  =  -(Hat(p_imu) * C)^T
    const Eigen::Vector3f A = Hat(p_imu) * C;
    H.block<1, 3>(i, kRotIdx) = -A.transpose();

    // Position block (cols kPosIdx … kPosIdx+2):
    //   Row = -n^T
    H.block<1, 3>(i, kPosIdx) = -normal.transpose();

    // Columns 6–17 (velocity, gyro bias, acc bias, gravity) remain zero —
    // LiDAR measurements carry no information about those states.
  }
}

void recompute_residuals_only(
    const LioState& state,
    const Se3& T_body_lidar,
    const std::vector<Correspondence>& corrs,
    Eigen::VectorXf& residuals) {
  const int n = static_cast<int>(corrs.size());
  residuals.resize(n);

  if (n == 0) return;

  const Eigen::Matrix3f R_il = T_body_lidar.rotation().matrix();
  const Eigen::Vector3f t_il = T_body_lidar.translation();
  const Eigen::Matrix3f R_wb = state.rotation;
  const Eigen::Vector3f t_wb = state.position;

  for (int i = 0; i < n; ++i) {
    const Eigen::Vector3f p_imu = R_il * corrs[i].p_lidar + t_il;
    const Eigen::Vector3f p_world = R_wb * p_imu + t_wb;
    residuals(i) = -(corrs[i].normal.dot(p_world) - corrs[i].plane_d);
  }
}

// ---------------------------------------------------------------------------
// Optimized: Nx6 Jacobian + pre-computed p_imu + cross product
// ---------------------------------------------------------------------------

void compute_lidar_jacobians_fast(
    const LioState& state,
    const std::vector<Eigen::Vector3f>& p_imu_cache,
    const std::vector<Correspondence>& corrs,
    Eigen::MatrixXf& H6,
    Eigen::VectorXf& residuals) {
  const int n = static_cast<int>(corrs.size());

  H6.resize(n, 6);
  residuals.resize(n);
  if (n == 0) return;

  const Eigen::Matrix3f R_wb = state.rotation;
  const Eigen::Vector3f t_wb = state.position;
  // Pre-compute R_wb^T once (used for all correspondences).
  const Eigen::Matrix3f R_wb_t = R_wb.transpose();

  // OMP threshold raised: for N < 1000, fork/join overhead (~50-100us)
  // exceeds parallel benefit. Sequential is faster for typical scan sizes.
  #pragma omp parallel for schedule(static) if(n > 1000)
  for (int i = 0; i < n; ++i) {
    const Eigen::Vector3f& p_imu  = p_imu_cache[i];
    const Eigen::Vector3f& normal = corrs[i].normal;
    const float            plane_d = corrs[i].plane_d;

    // body → world
    const Eigen::Vector3f p_world = R_wb * p_imu + t_wb;

    // Residual
    residuals(i) = -(normal.dot(p_world) - plane_d);

    // Jacobian: rotation block
    const Eigen::Vector3f C = R_wb_t * normal;
    const Eigen::Vector3f A = p_imu.cross(C);  // = Hat(p_imu) * C
    H6(i, 0) = -A(0);  H6(i, 1) = -A(1);  H6(i, 2) = -A(2);

    // Jacobian: position block
    H6(i, 3) = -normal(0);  H6(i, 4) = -normal(1);  H6(i, 5) = -normal(2);
  }
}

void recompute_residuals_fast(
    const LioState& state,
    const std::vector<Eigen::Vector3f>& p_imu_cache,
    const std::vector<Correspondence>& corrs,
    Eigen::VectorXf& residuals) {
  const int n = static_cast<int>(corrs.size());
  residuals.resize(n);
  if (n == 0) return;

  const Eigen::Matrix3f R_wb = state.rotation;
  const Eigen::Vector3f t_wb = state.position;

  // OMP threshold raised: for N < 1000, fork/join overhead (~50-100us)
  // exceeds parallel benefit. Sequential is faster for typical scan sizes.
  #pragma omp parallel for schedule(static) if(n > 1000)
  for (int i = 0; i < n; ++i) {
    const Eigen::Vector3f p_world = R_wb * p_imu_cache[i] + t_wb;
    residuals(i) = -(corrs[i].normal.dot(p_world) - corrs[i].plane_d);
  }
}

}  // namespace core
}  // namespace tof_slam
