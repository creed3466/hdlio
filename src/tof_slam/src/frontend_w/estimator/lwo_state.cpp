// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "tof_slam/frontend_w/estimator/lwo_state.hpp"

#include <ostream>

#include "tof_slam/common/lie/so3.hpp"

namespace tof_slam {
namespace lwo {

// ---------------------------------------------------------------------------
// Manifold operators
// ---------------------------------------------------------------------------

LwoState LwoState::operator+(const LwoStateVector& delta) const {
  LwoState result = *this;

  // Rotation: R_new = R * Exp(delta_rot)  (right perturbation on SO(3))
  const core::So3 dR = core::So3::Exp(delta.segment<3>(kLwoRotIdx));
  result.rotation = (core::So3(rotation) * dR).matrix();

  // Euclidean components: simple addition.
  result.position += delta.segment<3>(kLwoPosIdx);
  result.velocity += delta.segment<3>(kLwoVelIdx);
  result.wheel_scale += delta(kLwoScaleIdx);
  result.wheel_gyro_bias += delta.segment<2>(kLwoBiasIdx);
  result.ext_delta_yaw   += delta(kLwoExtYawIdx);
  result.ext_delta_xy    += delta.segment<2>(kLwoExtXyIdx);

  return result;
}

LwoStateVector LwoState::operator-(const LwoState& other) const {
  LwoStateVector delta;

  // Rotation: Log(R_other^T * R_this)
  const Eigen::Matrix3f R_diff = other.rotation.transpose() * rotation;
  delta.segment<3>(kLwoRotIdx) = core::So3(R_diff).Log();

  delta.segment<3>(kLwoPosIdx) = position - other.position;
  delta.segment<3>(kLwoVelIdx) = velocity - other.velocity;
  delta(kLwoScaleIdx)           = wheel_scale - other.wheel_scale;
  delta.segment<2>(kLwoBiasIdx) = wheel_gyro_bias - other.wheel_gyro_bias;
  delta(kLwoExtYawIdx)          = ext_delta_yaw - other.ext_delta_yaw;
  delta.segment<2>(kLwoExtXyIdx) = ext_delta_xy - other.ext_delta_xy;

  return delta;
}

LwoState& LwoState::operator+=(const LwoStateVector& delta) {
  *this = *this + delta;
  return *this;
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------

core::Se3 LwoState::pose() const {
  return core::Se3(core::So3(rotation), position);
}

void LwoState::set_pose(const core::Se3& T) {
  rotation = T.rotation_matrix();
  position = T.translation();
}

void LwoState::reset() {
  rotation = Eigen::Matrix3f::Identity();
  position = Eigen::Vector3f::Zero();
  velocity = Eigen::Vector3f::Zero();
  wheel_scale = 1.0f;
  wheel_gyro_bias = Eigen::Vector2f::Zero();
  ext_delta_yaw = 0.0f;
  ext_delta_xy = Eigen::Vector2f::Zero();

  covariance = LwoStateCovariance::Zero();
  // Rotation: small uncertainty (robot starts level).
  covariance.block<3, 3>(kLwoRotIdx, kLwoRotIdx) =
      Eigen::Matrix3f::Identity() * 1e-4f;
  // Position: moderate uncertainty.
  covariance.block<3, 3>(kLwoPosIdx, kLwoPosIdx) =
      Eigen::Matrix3f::Identity() * 0.01f;
  // Velocity: moderate uncertainty.
  covariance.block<3, 3>(kLwoVelIdx, kLwoVelIdx) =
      Eigen::Matrix3f::Identity() * 0.01f;
  // Scale: very tight (trust encoder calibration, scale ≈ 1.0).
  covariance(kLwoScaleIdx, kLwoScaleIdx) = 1e-5f;
  // Bias: frozen at zero (encoder is well-calibrated, bias unobservable
  // from encoder-only yaw — would need independent heading sensor).
  covariance.block<2, 2>(kLwoBiasIdx, kLwoBiasIdx) =
      Eigen::Matrix2f::Identity() * 1e-12f;
  // Extrinsic deltas: moderate initial uncertainty.
  // When ext calibration is disabled, these are never updated, so the value
  // only matters for P_inv conditioning.  Use same order as position (0.01)
  // to avoid worsening the condition number of the 15x15 P matrix.
  covariance(kLwoExtYawIdx, kLwoExtYawIdx) = 0.01f;
  covariance.block<2, 2>(kLwoExtXyIdx, kLwoExtXyIdx) =
      Eigen::Matrix2f::Identity() * 0.01f;
}

// ---------------------------------------------------------------------------
// Output
// ---------------------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const LwoState& s) {
  os << "LwoState {\n"
     << "  position:        [" << s.position.transpose() << "]\n"
     << "  velocity:        [" << s.velocity.transpose() << "]\n"
     << "  wheel_scale:     " << s.wheel_scale << "\n"
     << "  wheel_gyro_bias: [" << s.wheel_gyro_bias.transpose() << "]\n"
     << "  ext_delta_yaw:   " << s.ext_delta_yaw << "\n"
     << "  ext_delta_xy:    [" << s.ext_delta_xy.transpose() << "]\n"
     << "}";
  return os;
}

}  // namespace lwo
}  // namespace tof_slam
