"""
lwo_magok_caffe_launch.py — Run TofSLAM LWO frontend on magok_caffe dataset.

LWO mode: wheel odometry ON, IMU OFF.
Default: play full bag. Use timeout to limit (kills after N seconds of wall time).

Usage:
    ros2 launch tof_slam lwo_magok_caffe_launch.py
    ros2 launch tof_slam lwo_magok_caffe_launch.py bag_path:=/workspace/data/magok_caffe/magok_w10_f3
    ros2 launch tof_slam lwo_magok_caffe_launch.py rate:=0.5
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, TimerAction, OpaqueFunction, LogInfo,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('tof_slam')

    config   = LaunchConfiguration('config').perform(context)
    bag_path = LaunchConfiguration('bag_path').perform(context)
    rate     = LaunchConfiguration('rate').perform(context)

    nodes = []

    # tofslam_node — TofSLAM LWO frontend SLAM node
    nodes.append(Node(
        package='tof_slam',
        executable='tofslam_node',
        name='tofslam_node',
        output='screen',
        parameters=[config],
    ))

    # Bag playback (delayed 3 s to allow node startup)
    bag_cmd = [
        'ros2', 'bag', 'play', bag_path,
        '--rate', rate,
        '--clock',
        '--read-ahead-queue-size', '1000',
        '--topics',
        '/livox/lidar_192_168_0_65',
        '/odom',
        '/tf_static',
    ]

    nodes.append(TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=bag_cmd,
                output='screen',
            ),
        ],
    ))

    return nodes


def generate_launch_description():
    pkg_dir = get_package_share_directory('tof_slam')
    default_config = os.path.join(
        pkg_dir, 'config', 'lwo_magok_caffe.yaml')
    default_bag = '/workspace/data/magok_caffe/magok_w10_f3'

    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=default_config,
            description='Path to YAML configuration file'),
        DeclareLaunchArgument(
            'bag_path', default_value=default_bag,
            description='Path to the ROS 2 bag directory'),
        DeclareLaunchArgument(
            'rate', default_value='1.0',
            description='Bag playback rate multiplier'),

        LogInfo(msg=['TofSLAM LWO magok_caffe launch']),
        OpaqueFunction(function=launch_setup),
    ])
