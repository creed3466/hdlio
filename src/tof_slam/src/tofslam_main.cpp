/**
 * @file      tofslam_main.cpp
 * @brief     Entry point for tofslam_node — TofSLAM frontend pipeline.
 */

#include <rclcpp/rclcpp.hpp>
#include "tof_slam/ros/slam_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tof_slam::SlamNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
