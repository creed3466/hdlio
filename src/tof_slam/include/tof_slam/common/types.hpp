#pragma once

#include <cstdint>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace tof_slam {

/// Robot pose of body in world frame with covariance [rot; trans]
struct PoseState {
  Eigen::Quaterniond q_wb{Eigen::Quaterniond::Identity()};  // body orientation in world
  Eigen::Vector3d p_wb{Eigen::Vector3d::Zero()};            // body position in world
  Eigen::Matrix<double, 6, 6> P{Eigen::Matrix<double, 6, 6>::Zero()};  // covariance (6x6, rotation first then translation)
  double stamp{0.0};  // seconds
};

/// Wheel odometry relative motion between two timestamps [Implement_Plan.md §5.1]
struct WheelPrior {
  Eigen::Quaterniond q_bbnext{Eigen::Quaterniond::Identity()};  // relative rotation
  Eigen::Vector3d p_bbnext{Eigen::Vector3d::Zero()};            // relative translation
  Eigen::Matrix<double, 6, 6> Q{Eigen::Matrix<double, 6, 6>::Zero()};  // process noise (anisotropic)
  double slip_score{0.0};
  double t0{0.0};  // seconds
  double t1{0.0};  // seconds
};

/// Result of scan-to-map registration [Implement_Plan.md §5.1]
struct RegistrationResult {
  bool accepted{false};
  Eigen::Quaterniond q_correction{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d p_correction{Eigen::Vector3d::Zero()};
  Eigen::Matrix<double, 6, 6> P_post{Eigen::Matrix<double, 6, 6>::Zero()};
  double fitness{0.0};
  double inlier_ratio{0.0};
  double min_eigenvalue{0.0};
  uint8_t weak_axis_mask{0};  // bitmask for degenerate directions
};

/// Surface element for map representation [Implement_Plan.md §5.1]
struct Surfel {
  Eigen::Vector3d centroid{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d covariance{Eigen::Matrix3d::Zero()};
  Eigen::Vector3d normal{Eigen::Vector3d::UnitZ()};
  int support_count{0};
  double timestamp{0.0};
};

/// Single IMU measurement sample [Architecture.md §13]
struct ImuSample {
  double stamp{0.0};  // seconds
  Eigen::Vector3d gyro{Eigen::Vector3d::Zero()};   // rad/s, body frame
  Eigen::Vector3d accel{Eigen::Vector3d::Zero()};  // m/s^2, body frame
};

/// Extended state for per-point EKF (Point-LIO state_output parity)
///
/// State layout (30D manifold, 27D error-state):
///   [pos(3), rot(SO3→3), ext_R(SO3→3), ext_T(3), vel(3), omg(3), acc(3), bg(3), ba(3)]
///
/// The first 12 error-state dimensions correspond to H's nonzero columns
/// (pos, rot, extrinsic rotation, extrinsic translation), matching Point-LIO's
/// h_model structure. This enables the 12×12 subblock information filter.
///
/// omg and acc are state variables — IMU is treated as measurement (Point-LIO core).
struct EkfState {
  // Error-state index constants (27D tangent space)
  // Layout chosen so H's 12 nonzero columns are contiguous at [0:12).
  static constexpr int kPosIdx    = 0;   // position (world)
  static constexpr int kRotIdx    = 3;   // rotation (world←body)
  static constexpr int kExtRotIdx = 6;   // extrinsic rotation (body←sensor)
  static constexpr int kExtTrnIdx = 9;   // extrinsic translation (body←sensor)
  static constexpr int kVelIdx    = 12;  // velocity (world)
  static constexpr int kOmgIdx    = 15;  // angular velocity (body)
  static constexpr int kAccIdx    = 18;  // acceleration (body)
  static constexpr int kBgIdx     = 21;  // gyro bias
  static constexpr int kBaIdx     = 24;  // accel bias
  static constexpr int kStateDim  = 27;

  static constexpr int kHDim = 12;  // nonzero columns in H (pos+rot+ext_R+ext_T)

  Eigen::Quaterniond q_wb{Eigen::Quaterniond::Identity()};  // world←body rotation
  Eigen::Vector3d p_wb{Eigen::Vector3d::Zero()};            // world position
  Eigen::Quaterniond q_bs{Eigen::Quaterniond::Identity()};  // body←sensor rotation
  Eigen::Vector3d p_bs{Eigen::Vector3d::Zero()};            // body←sensor translation
  Eigen::Vector3d v_wb{Eigen::Vector3d::Zero()};            // velocity (world)
  Eigen::Vector3d omg{Eigen::Vector3d::Zero()};             // angular velocity (body, rad/s)
  Eigen::Vector3d acc{Eigen::Vector3d::Zero()};             // specific force (body, m/s^2)
  Eigen::Vector3d bg{Eigen::Vector3d::Zero()};              // gyroscope bias (rad/s)
  Eigen::Vector3d ba{Eigen::Vector3d::Zero()};              // accelerometer bias (m/s^2)
  Eigen::Matrix<double, 27, 27> P{Eigen::Matrix<double, 27, 27>::Identity() * 0.001};
  double stamp{0.0};  // seconds
  int update_count{0};
};

}  // namespace tof_slam
