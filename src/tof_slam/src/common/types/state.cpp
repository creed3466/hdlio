// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.

#include "tof_slam/common/types/state.hpp"

#include <ostream>

namespace tof_slam {
namespace core {

// ---------------------------------------------------------------------------
// Manifold operators
// ---------------------------------------------------------------------------

LioState LioState::operator+(const StateVector& delta) const {
  LioState result = *this;

  // Rotation: R_new = R * Exp(delta_rot)
  const So3 dR = So3::Exp(delta.segment<3>(kRotIdx));
  result.rotation = (So3(rotation) * dR).matrix();

  // Euclidean components: simple addition.
  result.position  += delta.segment<3>(kPosIdx);
  result.velocity  += delta.segment<3>(kVelIdx);
  result.gyro_bias += delta.segment<3>(kGyrBiasIdx);
  result.acc_bias  += delta.segment<3>(kAccBiasIdx);
  result.gravity   += delta.segment<3>(kGravIdx);

  return result;
}

StateVector LioState::operator-(const LioState& other) const {
  StateVector delta;

  // Rotation: Log(R_other^T * R_this)
  const Eigen::Matrix3f R_diff = other.rotation.transpose() * rotation;
  delta.segment<3>(kRotIdx) = So3(R_diff).Log();

  delta.segment<3>(kPosIdx)     = position  - other.position;
  delta.segment<3>(kVelIdx)     = velocity  - other.velocity;
  delta.segment<3>(kGyrBiasIdx) = gyro_bias - other.gyro_bias;
  delta.segment<3>(kAccBiasIdx) = acc_bias  - other.acc_bias;
  delta.segment<3>(kGravIdx)    = gravity   - other.gravity;

  return delta;
}

LioState& LioState::operator+=(const StateVector& delta) {
  *this = *this + delta;
  return *this;
}

// ---------------------------------------------------------------------------
// Accessors
// ---------------------------------------------------------------------------

Se3 LioState::pose() const {
  return Se3(So3(rotation), position);
}

void LioState::set_pose(const Se3& T) {
  rotation = T.rotation_matrix();
  position = T.translation();
}

void LioState::reset() {
  rotation = Eigen::Matrix3f::Identity();
  position = Eigen::Vector3f::Zero();
  velocity = Eigen::Vector3f::Zero();
  gyro_bias = Eigen::Vector3f::Zero();
  acc_bias = Eigen::Vector3f::Zero();
  gravity = Eigen::Vector3f(0.0f, 0.0f, -kGravityNorm);

  covariance = StateCovariance::Identity() * 0.01;
  // Tighter initial uncertainty for rotation and biases.
  covariance.block<3, 3>(kRotIdx, kRotIdx) =
      Eigen::Matrix3d::Identity() * 1e-5;
  covariance.block<9, 9>(kGyrBiasIdx, kGyrBiasIdx) =
      Eigen::Matrix<double, 9, 9>::Identity() * 1e-5;
}

// ---------------------------------------------------------------------------
// Output
// ---------------------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const LioState& s) {
  os << "LioState {\n"
     << "  position:  [" << s.position.transpose() << "]\n"
     << "  velocity:  [" << s.velocity.transpose() << "]\n"
     << "  gyro_bias: [" << s.gyro_bias.transpose() << "]\n"
     << "  acc_bias:  [" << s.acc_bias.transpose() << "]\n"
     << "  gravity:   [" << s.gravity.transpose() << "]\n"
     << "}";
  return os;
}

}  // namespace core
}  // namespace tof_slam
