#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace tof_slam {
namespace se3 {

/// Exponential map SO(3): rotation vector → rotation matrix
/// If dt != 1.0, computes Exp(omega * dt)
Eigen::Matrix3d Exp(const Eigen::Vector3d& omega, double dt = 1.0);

/// Logarithmic map SO(3): rotation matrix → rotation vector
Eigen::Vector3d Log(const Eigen::Matrix3d& R);

/// Skew-symmetric (hat) operator: R^3 → so(3)
Eigen::Matrix3d hat(const Eigen::Vector3d& v);

/// Vee operator: so(3) → R^3
Eigen::Vector3d vee(const Eigen::Matrix3d& S);

/// Right Jacobian of SO(3): J_r(phi) where phi = omega * dt
/// Used in ESKF covariance propagation
/// Formula: J_r = I - (1-cos||phi||)/||phi||^2 * [phi]_x + (1-sin||phi||/||phi||)/||phi||^2 * [phi]_x^2
Eigen::Matrix3d RightJacobian(const Eigen::Vector3d& omega, double dt = 1.0);

/// Left Jacobian of SO(3)
Eigen::Matrix3d LeftJacobian(const Eigen::Vector3d& v);

// ---- SE(3) rigid-body transform utilities ----

/// Build a 4×4 homogeneous transform from a quaternion and translation.
Eigen::Matrix4d toTransform(const Eigen::Quaterniond& q, const Eigen::Vector3d& p);

/// Extract quaternion and translation from a 4×4 homogeneous transform.
void fromTransform(const Eigen::Matrix4d& T, Eigen::Quaterniond& q, Eigen::Vector3d& p);

/// Invert an SE(3) transform (R, t) → (R^T, -R^T*t).
Eigen::Matrix4d inverseSE3(const Eigen::Matrix4d& T);

// ---- Aliases for compatibility with code using LogSO3 / LogSE3 naming ----

/// Alias: SO(3) log map (same as Log).
inline Eigen::Vector3d LogSO3(const Eigen::Matrix3d& R) { return Log(R); }

/// SE(3) log map: 4×4 matrix → 6D twist vector [rho; phi].
Eigen::Matrix<double, 6, 1> LogSE3(const Eigen::Matrix4d& T);

/// SE(3) exp map: 6D twist vector → 4×4 matrix.
Eigen::Matrix4d ExpSE3(const Eigen::Matrix<double, 6, 1>& xi);

}  // namespace se3
}  // namespace tof_slam
