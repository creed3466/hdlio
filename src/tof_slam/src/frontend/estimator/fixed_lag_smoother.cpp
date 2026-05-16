// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// fixed_lag_smoother.cpp — 2-frame velocity-coupled Gauss-Newton smoother.
//
// Joint state: [x_{k-1}(9), x_k(9)] = 18-dim
//   Per frame: delta_theta(3), delta_p(3), delta_v(3)
//
// Factors:
//   (a) Prior on x_{k-1}: from P_{k-1}^+ (9×9 top-left)
//   (b) Prior on x_k:     from P_k^+     (9×9 top-left)
//   (c) LiDAR factor for k-1: re-queried correspondences (new info)
//   (d) IMU preintegration factor: cross-frame coupling
//
// Frame k's LiDAR factor is NOT included — it's already absorbed into P_k^+.

#include "tof_slam/frontend/estimator/fixed_lag_smoother.hpp"

#include <cmath>

#include <Eigen/Dense>
#include <spdlog/spdlog.h>

#include "tof_slam/common/lie/so3.hpp"
#include "tof_slam/common/lie/se3.hpp"

namespace tof_slam {
namespace core {

// ---------------------------------------------------------------------------
// Internal: compute LiDAR factor contribution (6-DOF point-to-plane)
// ---------------------------------------------------------------------------
//
// For each correspondence, the Jacobian h = [-A^T, -n^T] (1×6) in
// the pose tangent space, where A = p_imu × (R_wb^T * n).
// Extended to 1×9 by appending zero velocity columns.
//
// Returns H^T W H (9×9) and H^T W r (9×1).

static void accumulate_lidar_factor(
    const LioState& state,
    const Se3& T_body_lidar,
    const std::vector<Correspondence>& corrs,
    float lidar_noise_std,
    Eigen::Matrix<double, 9, 9>& H_out,
    Eigen::Matrix<double, 9, 1>& b_out) {
  H_out.setZero();
  b_out.setZero();

  if (corrs.empty()) return;

  const Eigen::Matrix3f R_wb = state.rotation;
  const Eigen::Matrix3f R_bw = R_wb.transpose();
  const Eigen::Vector3f p_wb = state.position;
  const Eigen::Matrix3f R_bl = T_body_lidar.rotation_matrix();
  const Eigen::Vector3f t_bl = T_body_lidar.translation();

  const double sigma2 = static_cast<double>(lidar_noise_std * lidar_noise_std);
  const double w = 1.0 / sigma2;

  for (const auto& c : corrs) {
    // Transform point to body frame.
    const Eigen::Vector3f p_body = R_bl * c.p_lidar + t_bl;
    // Transform to world frame.
    const Eigen::Vector3f p_world = R_wb * p_body + p_wb;
    // Surfel normal.
    const Eigen::Vector3f n = c.normal;

    // Residual: negated signed point-to-plane distance (matches IEKF convention).
    // plane_d = n^T * centroid, so r = -(n^T * p_world - n^T * centroid).
    const float residual = -(n.dot(p_world) - c.plane_d);

    // Jacobian (1×6): h = [-A^T, -n^T] where A = p_body × (R_bw * n)
    const Eigen::Vector3f Rn = R_bw * n;
    const Eigen::Vector3f A = p_body.cross(Rn);

    // 1×9 Jacobian (rotation, position, velocity=0)
    Eigen::Matrix<double, 1, 9> h;
    h << -A(0), -A(1), -A(2),
         -n(0), -n(1), -n(2),
         0.0, 0.0, 0.0;

    const double r = static_cast<double>(residual);

    // Accumulate H^T W H and H^T W r.
    H_out.noalias() += w * h.transpose() * h;
    b_out.noalias() += w * r * h.transpose();
  }
}

// ---------------------------------------------------------------------------
// Internal: compute IMU preintegration factor contribution
// ---------------------------------------------------------------------------
//
// Residual r_imu (9×1):
//   r_rot = Log(delta_R^T * R_i^T * R_j)
//   r_vel = R_i^T * (v_j - v_i - g * dt) - delta_v
//   r_pos = R_i^T * (p_j - p_i - v_i * dt - 0.5 * g * dt²) - delta_p
//
// Jacobians J_prev (9×9) and J_curr (9×9) produce the cross-frame coupling.

static void accumulate_imu_factor(
    const LioState& state_prev,
    const LioState& state_curr,
    const ImuPreintegration& preint,
    Eigen::Matrix<double, 9, 9>& H_00,
    Eigen::Matrix<double, 9, 9>& H_01,
    Eigen::Matrix<double, 9, 9>& H_11,
    Eigen::Matrix<double, 9, 1>& b_0,
    Eigen::Matrix<double, 9, 1>& b_1) {
  const Eigen::Matrix3f R_i = state_prev.rotation;
  const Eigen::Matrix3f R_j = state_curr.rotation;
  const Eigen::Vector3f p_i = state_prev.position;
  const Eigen::Vector3f p_j = state_curr.position;
  const Eigen::Vector3f v_i = state_prev.velocity;
  const Eigen::Vector3f v_j = state_curr.velocity;
  const Eigen::Vector3f g = state_prev.gravity;
  const float dt = static_cast<float>(preint.delta_t);

  const Eigen::Matrix3f R_iT = R_i.transpose();

  // --- Bias-corrected preintegrated deltas ---
  // First-order correction for bias change from linearization point.
  const Eigen::Vector3f dbg = state_prev.gyro_bias - preint.bg_lin;
  const Eigen::Vector3f dba = state_prev.acc_bias - preint.ba_lin;

  const Eigen::Matrix3f dR_corr =
      preint.delta_R * So3::Exp(preint.J_bg_rot * dbg).matrix();
  const Eigen::Vector3f dv_corr =
      preint.delta_v + preint.J_bg_vel * dbg + preint.J_ba_vel * dba;
  const Eigen::Vector3f dp_corr =
      preint.delta_p + preint.J_bg_pos * dbg + preint.J_ba_pos * dba;

  // --- Residuals ---
  // Rotation residual: Log(dR_corr^T * R_i^T * R_j)
  const Eigen::Matrix3f err_R = dR_corr.transpose() * R_iT * R_j;
  const Eigen::Vector3f r_rot = So3(err_R).Log();

  // Velocity residual: R_i^T * (v_j - v_i - g * dt) - dv_corr
  const Eigen::Vector3f dv_world = v_j - v_i - g * dt;
  const Eigen::Vector3f r_vel = R_iT * dv_world - dv_corr;

  // Position residual: R_i^T * (p_j - p_i - v_i*dt - 0.5*g*dt²) - dp_corr
  const Eigen::Vector3f dp_world =
      p_j - p_i - v_i * dt - 0.5f * g * dt * dt;
  const Eigen::Vector3f r_pos = R_iT * dp_world - dp_corr;

  // --- Jacobians ---
  // Right Jacobian inverse for rotation error.
  // Jr_inv(r_rot) is needed for the rotation Jacobian.
  // For small r_rot (typical case), Jr^{-1} ≈ I + 0.5 * [r_rot]_x.
  // Use full formula via existing RightJacobian and inversion.
  // For robustness, use the approximation for small angles.
  const float r_rot_norm = r_rot.norm();
  Eigen::Matrix3f Jr_inv;
  if (r_rot_norm < 1e-5f) {
    Jr_inv = Eigen::Matrix3f::Identity() + 0.5f * Hat(r_rot);
  } else {
    // Full formula: Jr^{-1} = I + 0.5*[phi]_x + (1/theta^2) *
    //   (1 - theta*sin(theta) / (2*(1-cos(theta)))) * [phi]_x^2
    const float th = r_rot_norm;
    const Eigen::Matrix3f phi_hat = Hat(r_rot);
    const float half_th = 0.5f * th;
    const float cot_half = half_th / std::tan(half_th);
    Jr_inv = Eigen::Matrix3f::Identity()
             + 0.5f * phi_hat
             + (1.0f / (th * th)) * (1.0f - cot_half) * phi_hat * phi_hat;
  }

  // J_prev (9×9): Jacobian of r_imu w.r.t. x_{k-1} = (theta_i, p_i, v_i)
  // Ordering: [rot(3), pos(3), vel(3)]
  //
  // d(r_rot)/d(theta_i) = -Jr_inv * dR_corr^T    (3×3)
  // d(r_rot)/d(p_i)     = 0                       (3×3)
  // d(r_rot)/d(v_i)     = 0                       (3×3)
  //
  // d(r_vel)/d(theta_i) = [R_i^T * dv_world]_×   (3×3)  (from d(R_i^T)/d(theta_i))
  // d(r_vel)/d(p_i)     = 0                       (3×3)
  // d(r_vel)/d(v_i)     = -R_i^T                  (3×3)
  //
  // d(r_pos)/d(theta_i) = [R_i^T * dp_world]_×   (3×3)
  // d(r_pos)/d(p_i)     = -R_i^T                  (3×3)
  // d(r_pos)/d(v_i)     = -R_i^T * dt             (3×3)

  Eigen::Matrix<double, 9, 9> J_prev = Eigen::Matrix<double, 9, 9>::Zero();
  // rot block
  J_prev.block<3, 3>(0, 0) =
      (-Jr_inv * dR_corr.transpose()).cast<double>();
  // vel block
  J_prev.block<3, 3>(3, 0) = Hat(R_iT * dv_world).cast<double>();
  J_prev.block<3, 3>(3, 6) = (-R_iT).cast<double>();
  // pos block
  J_prev.block<3, 3>(6, 0) = Hat(R_iT * dp_world).cast<double>();
  J_prev.block<3, 3>(6, 3) = (-R_iT).cast<double>();
  J_prev.block<3, 3>(6, 6) = (-R_iT * dt).cast<double>();

  // J_curr (9×9): Jacobian of r_imu w.r.t. x_k = (theta_j, p_j, v_j)
  //
  // d(r_rot)/d(theta_j) = Jr_inv                  (3×3)
  // d(r_rot)/d(p_j)     = 0                       (3×3)
  // d(r_rot)/d(v_j)     = 0                       (3×3)
  //
  // d(r_vel)/d(theta_j) = 0                       (3×3)
  // d(r_vel)/d(p_j)     = 0                       (3×3)
  // d(r_vel)/d(v_j)     = R_i^T                   (3×3)
  //
  // d(r_pos)/d(theta_j) = 0                       (3×3)
  // d(r_pos)/d(p_j)     = R_i^T                   (3×3)
  // d(r_pos)/d(v_j)     = 0                       (3×3)

  Eigen::Matrix<double, 9, 9> J_curr = Eigen::Matrix<double, 9, 9>::Zero();
  J_curr.block<3, 3>(0, 0) = Jr_inv.cast<double>();
  J_curr.block<3, 3>(3, 6) = R_iT.cast<double>();
  J_curr.block<3, 3>(6, 3) = R_iT.cast<double>();

  // --- Preintegration covariance inverse ---
  // Regularize for numerical stability.
  Eigen::Matrix<double, 9, 9> Q_inv = preint.covariance;
  for (int i = 0; i < 9; ++i) {
    Q_inv(i, i) = std::max(Q_inv(i, i), 1e-10);
  }
  Q_inv = Q_inv.inverse().eval();

  // --- Accumulate into 18×18 Hessian blocks ---
  // H_imu = J_full^T * Q^{-1} * J_full
  // where J_full = [J_prev | J_curr] (9×18)
  //
  // H_imu = | J_prev^T Q^{-1} J_prev    J_prev^T Q^{-1} J_curr  |
  //         | J_curr^T Q^{-1} J_prev    J_curr^T Q^{-1} J_curr  |

  H_00.noalias() += J_prev.transpose() * Q_inv * J_prev;
  H_01.noalias() += J_prev.transpose() * Q_inv * J_curr;
  H_11.noalias() += J_curr.transpose() * Q_inv * J_curr;

  // Residual vector.
  Eigen::Matrix<double, 9, 1> r_imu;
  r_imu << r_rot.cast<double>(), r_vel.cast<double>(), r_pos.cast<double>();

  b_0.noalias() += J_prev.transpose() * Q_inv * r_imu;
  b_1.noalias() += J_curr.transpose() * Q_inv * r_imu;
}

// ---------------------------------------------------------------------------
// fixed_lag_smooth
// ---------------------------------------------------------------------------

FlsResult fixed_lag_smooth(
    const LioState& state_prev,
    const LioState& state_curr,
    const StateCovariance& P_prev,
    const StateCovariance& P_curr,
    const std::vector<Correspondence>& corrs_prev,
    const ImuPreintegration& preint,
    const Se3& T_body_lidar,
    const IekfConfig& iekf_config,
    const FlsConfig& fls_config) {
  FlsResult result;
  result.state_curr = state_curr;

  if (!preint.valid() || corrs_prev.empty()) {
    return result;
  }

  // Working copies of states (modified during iterations).
  LioState x_prev = state_prev;
  LioState x_curr = state_curr;

  // Extract 9×9 prior information matrices from IEKF posteriors.
  // Only use rotation(0:3), position(3:6), velocity(6:9).
  Eigen::Matrix<double, 9, 9> P_prev_9 = P_prev.block<9, 9>(0, 0);
  Eigen::Matrix<double, 9, 9> P_curr_9 = P_curr.block<9, 9>(0, 0);

  // Eigenvalue-clamped inversion for numerical stability.
  // The IEKF posterior can have very small eigenvalues (especially when
  // many correspondences produce a tight posterior), and diagonal-only
  // floor clamping doesn't fix ill-conditioning from off-diagonal entries.
  // Eigenvalue clamping preserves the correlation structure while bounding
  // the condition number.
  auto eigenvalue_clamped_inverse = [](
      const Eigen::Matrix<double, 9, 9>& P,
      double min_eigenvalue) -> Eigen::Matrix<double, 9, 9> {
    // Symmetrize for numerical safety.
    const Eigen::Matrix<double, 9, 9> P_sym = 0.5 * (P + P.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 9, 9>> es(P_sym);
    if (es.info() != Eigen::Success) {
      SPDLOG_WARN("FLS: eigen decomposition failed, using diagonal prior");
      return P_sym.diagonal().cwiseMax(min_eigenvalue)
                 .asDiagonal().inverse();
    }
    // Clamp eigenvalues to minimum floor.
    const auto evals_clamped = es.eigenvalues().cwiseMax(min_eigenvalue);
    const auto& evecs = es.eigenvectors();
    const double cond = evals_clamped.maxCoeff() / evals_clamped.minCoeff();
    if (cond > 1e12) {
      SPDLOG_WARN("FLS: prior cond={:.1e} after clamping, using diagonal",
                  cond);
      return P_sym.diagonal().cwiseMax(min_eigenvalue)
                 .asDiagonal().inverse();
    }
    return evecs * evals_clamped.cwiseInverse().asDiagonal()
                 * evecs.transpose();
  };

  // Minimum eigenvalue: use the velocity floor (loosest) as a reasonable
  // lower bound for any direction in the 9-dim state space.
  const double min_eval = static_cast<double>(iekf_config.p_floor_vel);

  const Eigen::Matrix<double, 9, 9> info_prev =
      eigenvalue_clamped_inverse(P_prev_9, min_eval);
  const Eigen::Matrix<double, 9, 9> info_curr =
      eigenvalue_clamped_inverse(P_curr_9, min_eval);

  // --- Gauss-Newton iterations ---
  for (int iter = 0; iter < fls_config.max_iterations; ++iter) {
    // Initialize 18×18 Hessian blocks and 18×1 gradient.
    Eigen::Matrix<double, 9, 9> H_00 = Eigen::Matrix<double, 9, 9>::Zero();
    Eigen::Matrix<double, 9, 9> H_01 = Eigen::Matrix<double, 9, 9>::Zero();
    Eigen::Matrix<double, 9, 9> H_11 = Eigen::Matrix<double, 9, 9>::Zero();
    Eigen::Matrix<double, 9, 1> b_0 = Eigen::Matrix<double, 9, 1>::Zero();
    Eigen::Matrix<double, 9, 1> b_1 = Eigen::Matrix<double, 9, 1>::Zero();

    // (a) Prior factor for frame k-1.
    H_00 += info_prev;
    // Prior residual: dx_{k-1} = x_{k-1,current} - x_{k-1,iekf}
    // (In tangent space, at each iteration this is the accumulated correction.)
    {
      const Eigen::Vector3f dr = So3(state_prev.rotation.transpose() *
                                      x_prev.rotation).Log();
      const Eigen::Vector3f dp = x_prev.position - state_prev.position;
      const Eigen::Vector3f dv = x_prev.velocity - state_prev.velocity;
      Eigen::Matrix<double, 9, 1> dx_prev;
      dx_prev << dr.cast<double>(), dp.cast<double>(), dv.cast<double>();
      b_0 += info_prev * dx_prev;
    }

    // (b) Prior factor for frame k.
    H_11 += info_curr;
    {
      const Eigen::Vector3f dr = So3(state_curr.rotation.transpose() *
                                      x_curr.rotation).Log();
      const Eigen::Vector3f dp = x_curr.position - state_curr.position;
      const Eigen::Vector3f dv = x_curr.velocity - state_curr.velocity;
      Eigen::Matrix<double, 9, 1> dx_curr;
      dx_curr << dr.cast<double>(), dp.cast<double>(), dv.cast<double>();
      b_1 += info_curr * dx_curr;
    }

    // (c) LiDAR factor for frame k-1 (re-queried, new information).
    Eigen::Matrix<double, 9, 9> H_lidar;
    Eigen::Matrix<double, 9, 1> b_lidar;
    accumulate_lidar_factor(x_prev, T_body_lidar, corrs_prev,
                            iekf_config.lidar_noise_std, H_lidar, b_lidar);
    H_00 += H_lidar;
    b_0 += b_lidar;

    // (d) IMU preintegration factor (cross-frame coupling).
    accumulate_imu_factor(x_prev, x_curr, preint,
                          H_00, H_01, H_11, b_0, b_1);

    // Record coupling strength and diagnostic at iter 0.
    if (iter == 0) {
      result.cross_block_norm =
          static_cast<float>(H_01.norm());  // Frobenius norm

      // Diagnostic: log factor magnitudes and system conditioning.
      const double info_prev_norm = info_prev.norm();
      const double h_lidar_norm = H_lidar.norm();
      const double b0_norm = b_0.norm();
      const double b1_norm = b_1.norm();

      // Condition number of full 18×18 H.
      Eigen::Matrix<double, 18, 18> H_diag_check;
      H_diag_check.block<9, 9>(0, 0) = H_00;
      H_diag_check.block<9, 9>(0, 9) = H_01;
      H_diag_check.block<9, 9>(9, 0) = H_01.transpose();
      H_diag_check.block<9, 9>(9, 9) = H_11;
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 18, 18>> es_h(
          H_diag_check);
      const double h_min = es_h.eigenvalues().minCoeff();
      const double h_max = es_h.eigenvalues().maxCoeff();
      const double h_cond = h_max / std::max(h_min, 1e-30);

      SPDLOG_INFO("FLS diag: info_prev={:.1e} H_lidar={:.1e} H01={:.1e} "
                  "b0={:.1e} b1={:.1e} H_cond={:.1e} H_min={:.1e}",
                  info_prev_norm, h_lidar_norm, result.cross_block_norm,
                  b0_norm, b1_norm, h_cond, h_min);
    }

    // --- Assemble full 18×18 system ---
    Eigen::Matrix<double, 18, 18> H;
    H.block<9, 9>(0, 0) = H_00;
    H.block<9, 9>(0, 9) = H_01;
    H.block<9, 9>(9, 0) = H_01.transpose();
    H.block<9, 9>(9, 9) = H_11;

    Eigen::Matrix<double, 18, 1> b;
    b.head<9>() = b_0;
    b.tail<9>() = b_1;

    // --- Solve H * dx = -b ---
    const auto ldlt = H.ldlt();
    if (ldlt.info() != Eigen::Success) {
      SPDLOG_WARN("FLS: LDLT decomposition failed at iter {}", iter);
      break;
    }
    const Eigen::Matrix<double, 18, 1> dx = ldlt.solve(-b);

    // --- Apply update on manifold ---
    // Frame k-1: update rotation, position, velocity.
    {
      const Eigen::Vector3f dtheta = dx.segment<3>(0).cast<float>();
      x_prev.rotation = x_prev.rotation * So3::Exp(dtheta).matrix();
      x_prev.position += dx.segment<3>(3).cast<float>();
      x_prev.velocity += dx.segment<3>(6).cast<float>();
    }

    // Frame k: update rotation, position, velocity.
    {
      const Eigen::Vector3f dtheta = dx.segment<3>(9).cast<float>();
      x_curr.rotation = x_curr.rotation * So3::Exp(dtheta).matrix();
      x_curr.position += dx.segment<3>(12).cast<float>();
      x_curr.velocity += dx.segment<3>(15).cast<float>();
    }

    result.dx_norm_final = static_cast<float>(dx.norm());
    result.iterations = iter + 1;

    // Convergence check.
    if (result.dx_norm_final < fls_config.convergence_threshold) {
      result.converged = true;
      break;
    }
  }

  // --- Compute correction magnitude for frame k ---
  const Eigen::Vector3f dp_correction = x_curr.position - state_curr.position;
  result.pos_correction_curr = dp_correction.norm();

  const Eigen::Matrix3f dR_correction =
      state_curr.rotation.transpose() * x_curr.rotation;
  result.rot_correction_curr = So3(dR_correction).Log().norm();

  result.vel_correction_curr =
      (x_curr.velocity - state_curr.velocity).norm();

  // --- Safety cap: reject unreasonably large corrections ---
  if (result.pos_correction_curr > fls_config.max_pos_correction ||
      result.rot_correction_curr > fls_config.max_rot_correction ||
      result.vel_correction_curr > fls_config.max_vel_correction) {
    SPDLOG_WARN("FLS: reject — pos={:.4f}m rot={:.4f}deg vel={:.4f}m/s exceed caps",
                result.pos_correction_curr,
                result.rot_correction_curr * 180.0f / 3.14159265f,
                result.vel_correction_curr);
    result.state_curr = state_curr;  // revert
    result.applied = false;
    return result;
  }

  // Apply corrected state.
  result.state_curr = x_curr;
  result.applied = true;

  return result;
}

}  // namespace core
}  // namespace tof_slam
