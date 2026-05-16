#include "tof_slam/common/se3.hpp"
#include <cmath>

namespace tof_slam {
namespace se3 {

Eigen::Matrix3d Exp(const Eigen::Vector3d& omega, double dt) {
  Eigen::Vector3d phi = omega * dt;
  double angle = phi.norm();
  if (angle < 1e-8) return Eigen::Matrix3d::Identity();
  Eigen::Vector3d axis = phi / angle;
  // Rodrigues: R = I + sin(a)*K + (1-cos(a))*K^2
  Eigen::Matrix3d K = hat(axis);
  return Eigen::Matrix3d::Identity() + std::sin(angle) * K + (1.0 - std::cos(angle)) * K * K;
}

Eigen::Vector3d Log(const Eigen::Matrix3d& R) {
  // Use Eigen's AngleAxisd
  Eigen::AngleAxisd aa(R);
  double angle = aa.angle();
  if (angle < 1e-10) return Eigen::Vector3d::Zero();
  return aa.axis() * angle;
}

Eigen::Matrix3d hat(const Eigen::Vector3d& v) {
  Eigen::Matrix3d S;
  S <<     0.0, -v[2],  v[1],
         v[2],   0.0, -v[0],
        -v[1],  v[0],   0.0;
  return S;
}

Eigen::Vector3d vee(const Eigen::Matrix3d& S) {
  return Eigen::Vector3d(S(2, 1), S(0, 2), S(1, 0));
}

Eigen::Matrix3d RightJacobian(const Eigen::Vector3d& omega, double dt) {
  // This is EXACTLY the Super-LIO implementation from ESKF.cpp
  Eigen::Vector3d phi = omega * dt;
  double norm = phi.norm();
  if (norm < 1e-8) return Eigen::Matrix3d::Identity();
  Eigen::Vector3d axis = phi / norm;
  Eigen::Matrix3d K = hat(axis);
  double a = (1.0 - std::cos(norm)) / norm;
  double b = 1.0 - std::sin(norm) / norm;
  return Eigen::Matrix3d::Identity() - a * K + b * K * K;
}

Eigen::Matrix3d LeftJacobian(const Eigen::Vector3d& v) {
  double norm = v.norm();
  if (norm < 1e-6) return Eigen::Matrix3d::Identity();
  Eigen::Matrix3d K = hat(v);
  double sq = norm * norm;
  return Eigen::Matrix3d::Identity() + (1.0 - std::cos(norm)) / sq * K + (1.0 - std::sin(norm) / norm) / sq * K * K;
}

Eigen::Matrix4d toTransform(const Eigen::Quaterniond& q, const Eigen::Vector3d& p) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
  T.block<3, 1>(0, 3) = p;
  return T;
}

void fromTransform(const Eigen::Matrix4d& T, Eigen::Quaterniond& q, Eigen::Vector3d& p) {
  q = Eigen::Quaterniond(T.block<3, 3>(0, 0)).normalized();
  p = T.block<3, 1>(0, 3);
}

Eigen::Matrix4d inverseSE3(const Eigen::Matrix4d& T) {
  Eigen::Matrix4d T_inv = Eigen::Matrix4d::Identity();
  const Eigen::Matrix3d R_T = T.block<3, 3>(0, 0).transpose();
  const Eigen::Vector3d t   = T.block<3, 1>(0, 3);
  T_inv.block<3, 3>(0, 0) = R_T;
  T_inv.block<3, 1>(0, 3) = -R_T * t;
  return T_inv;
}

Eigen::Matrix<double, 6, 1> LogSE3(const Eigen::Matrix4d& T) {
  // Extract rotation and translation
  const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  const Eigen::Vector3d t = T.block<3, 1>(0, 3);

  // SO(3) log
  const Eigen::Vector3d phi = Log(R);
  const double angle = phi.norm();

  // Compute J_l^{-1} for the translation part
  // For small angles, J_l^{-1} ≈ I
  Eigen::Matrix3d J_l_inv = Eigen::Matrix3d::Identity();
  if (angle > 1e-8) {
    const Eigen::Matrix3d K = hat(phi / angle);
    // J_l = sin(a)/a * I + (1 - sin(a)/a) * axis*axis^T + (1-cos(a))/a * K — approximate J_l_inv
    // Use: J_l_inv ≈ I - 0.5*K + (1/angle^2 - (1+cos(a))/(2*sin(a)*angle))*K^2
    const double c = (1.0 / (angle * angle)) -
                     (1.0 + std::cos(angle)) / (2.0 * std::sin(angle) * angle);
    J_l_inv = Eigen::Matrix3d::Identity() - 0.5 * hat(phi) + c * K * K;
  }

  Eigen::Matrix<double, 6, 1> xi;
  xi.head<3>() = J_l_inv * t;  // rho
  xi.tail<3>() = phi;           // phi
  return xi;
}

Eigen::Matrix4d ExpSE3(const Eigen::Matrix<double, 6, 1>& xi) {
  const Eigen::Vector3d rho = xi.head<3>();
  const Eigen::Vector3d phi = xi.tail<3>();
  const double angle = phi.norm();

  Eigen::Matrix3d R;
  Eigen::Matrix3d J_l;
  if (angle < 1e-8) {
    R   = Eigen::Matrix3d::Identity() + hat(phi);
    J_l = Eigen::Matrix3d::Identity() + 0.5 * hat(phi);
  } else {
    const Eigen::Vector3d axis = phi / angle;
    const Eigen::Matrix3d K = hat(axis);
    R = Eigen::Matrix3d::Identity() +
        std::sin(angle) * K +
        (1.0 - std::cos(angle)) * K * K;
    J_l = (std::sin(angle) / angle) * Eigen::Matrix3d::Identity() +
          (1.0 - std::sin(angle) / angle) * axis * axis.transpose() +
          ((1.0 - std::cos(angle)) / angle) * K;
  }

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = J_l * rho;
  return T;
}

}  // namespace se3
}  // namespace tof_slam
