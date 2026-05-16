// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "tof_slam/frontend_w/measurement/wheel_measurement.hpp"

#include "tof_slam/common/lie/so3.hpp"

namespace tof_slam {
namespace lwo {

WheelMeasurement::WheelMeasurement(const WheelMeasurementConfig& cfg)
    : cfg_(cfg) {}

// ---------------------------------------------------------------------------
// noise
// ---------------------------------------------------------------------------

Eigen::Matrix2f WheelMeasurement::noise() const {
  const float sv = cfg_.noise_vx;
  const float sw = cfg_.noise_omega_z;
  return Eigen::DiagonalMatrix<float, 2>(sv * sv, sw * sw).toDenseMatrix();
}

// ---------------------------------------------------------------------------
// jacobian
// ---------------------------------------------------------------------------

Eigen::Matrix<float, 2, kLwoStateDim> WheelMeasurement::jacobian(
    const LwoState& state) const {

  Eigen::Matrix<float, 2, kLwoStateDim> H =
      Eigen::Matrix<float, 2, kLwoStateDim>::Zero();

  const Eigen::Matrix3f& R = state.rotation;
  const float s = state.wheel_scale;
  const Eigen::Vector3f& v = state.velocity;

  // v_x^body = e1^T * R^T * v
  const Eigen::Vector3f Rtv = R.transpose() * v;
  const float vx_body = Rtv(0);

  // ---- Row 0: d(r_1)/d(x) = d(v_x^enc - (1/s)*e1^T*R^T*v) / d(x) --------

  // d(r_1)/d(phi): SO(3) right-perturbation.
  //   d((1/s)*e1^T*R^T*v)/d(delta_phi) = -(1/s)*e1^T*d(R^T*v)/d(delta_phi)
  //   Using d(R^T*v)/d(delta_phi) = -[R^T*v]_x (right perturbation):
  //   => d((1/s)*e1^T*R^T*v)/d(delta_phi) = (1/s)*e1^T*[R^T*v]_x
  //   => d(r_1)/d(phi) = -(1/s)*e1^T*[R^T*v]_x  (negative because r = z - z_hat)
  //   But wait: r_1 = v_x^enc - z_hat, so dr_1/dx = -d(z_hat)/dx.
  //   d(z_hat_1)/d(phi) = (1/s)*e1^T*[R^T*v]_x
  //   => H[0, 0:3] = (1/s)*e1^T*[Rtv]_x   (sign: dr/dphi = -dz_hat/dphi,
  //      but Hat skew gives: d(R^T v)/d(delta_phi) = [R^T v]_x for right
  //      perturbation convention — see docs/02_lwo_design.md §3.4.3)
  //
  // Following the sign convention in 02_lwo_design.md (H = dr/dx):
  //   H[0, 0:3] = (1/s) * e1^T * [R^T*v]_x
  // e1^T * Hat(Rtv) = Hat(Rtv).row(0)
  const Eigen::Matrix3f skew_Rtv = core::Hat(Rtv);
  H.block<1, 3>(0, kLwoRotIdx) = (1.0f / s) * skew_Rtv.row(0);

  // d(r_1)/d(v) = -(1/s) * e1^T * R^T  (negative: r = z - z_hat)
  H.block<1, 3>(0, kLwoVelIdx) = -(1.0f / s) * R.col(0).transpose();  // e1^T * R^T = R.col(0)^T

  // d(r_1)/d(s) = vx_body / s^2  (from d(-vx_body/s)/ds = vx_body/s^2)
  H(0, kLwoScaleIdx) = vx_body / (s * s);

  // d(r_1)/d(b_omega_x) = 0, d(r_1)/d(b_omega_z) = 0  (already zero)

  // ---- Row 1: d(r_2)/d(x) = d(omega_z^enc - omega_z^b - b_omega_z) / d(x) -

  // d(r_2)/d(b_omega_z) = -1  (column 11 = kLwoBiasIdx+1)
  H(1, kLwoBiasIdx + 1) = -1.0f;

  return H;
}

// ---------------------------------------------------------------------------
// compute
// ---------------------------------------------------------------------------

WheelMeasurementResult WheelMeasurement::compute(const LwoState& state,
                                                   float vx_enc,
                                                   float omega_z_enc,
                                                   float omega_z_b) const {
  WheelMeasurementResult result;

  result.H         = jacobian(state);
  result.noise_cov = noise();

  const float s   = state.wheel_scale;
  const float b_oz = state.wheel_gyro_bias(1);

  // Body-frame forward velocity predicted from world-frame state.
  const float vx_body = state.rotation.col(0).dot(state.velocity);  // e1^T * R^T * v

  // Residuals: z - z_hat
  result.residual(0) = vx_enc  - vx_body / s;
  result.residual(1) = omega_z_enc - omega_z_b - b_oz;

  return result;
}

}  // namespace lwo
}  // namespace tof_slam
