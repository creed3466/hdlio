/**
 * @file      tofslam_main.cpp
 * @brief     Entry point for tofslam_node — TofSLAM frontend pipeline (ROS 1).
 */

#include <ros/ros.h>
#include "tof_slam/ros1/slam_node.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "tofslam_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    tof_slam::SlamNode node(nh, pnh);
    ros::spin();
    ros::shutdown();
    return 0;
}
