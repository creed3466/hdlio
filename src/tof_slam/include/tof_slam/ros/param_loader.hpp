// Copyright 2025 TofSLAM Authors. All rights reserved.
// Licensed under the MIT License.
//
// param_loader.hpp — Stateless free functions for ROS parameter loading.
//
// Extracts parameter declaration and config-building logic from SlamNode so
// that the node constructor remains a thin orchestrator.

#ifndef TOF_SLAM_ROS_PARAM_LOADER_HPP_
#define TOF_SLAM_ROS_PARAM_LOADER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

#include "tof_slam/frontend/estimator/lio_estimator.hpp"
#include "tof_slam/frontend_w/estimator/lwo_estimator.hpp"
#include "tof_slam/backend/loop_closure_manager.hpp"
#include "tof_slam/ros/imu_adapter.hpp"

namespace tof_slam {
namespace param_loader {

/// Declares all ROS parameters with defaults.
void declare_all_params(rclcpp::Node& node);

/// Parses extrinsic RPY or 3x3 matrix into a rotation matrix.
Eigen::Matrix3f build_extrinsic_rot(const rclcpp::Node& node);

/// Config + adapter config for LIO mode.
struct LioSetupResult {
  core::LioEstimator::Config estimator_config;
  ros_adapter::ImuAdapter::Config imu_adapter_config;
};

/// Builds LIO estimator and IMU adapter configs from node parameters.
LioSetupResult build_lio_config(const rclcpp::Node& node,
                                const Eigen::Matrix3f& ext_rot,
                                double ext_x, double ext_y, double ext_z);

/// Builds LWO estimator config from node parameters.
lwo::LwoEstimator::Config build_lwo_config(const rclcpp::Node& node,
                                           const Eigen::Matrix3f& ext_rot,
                                           double ext_x, double ext_y,
                                           double ext_z);

/// Builds loop closure config from node parameters.
LoopClosureConfig build_loop_closure_config(const rclcpp::Node& node);

}  // namespace param_loader
}  // namespace tof_slam

#endif  // TOF_SLAM_ROS_PARAM_LOADER_HPP_
