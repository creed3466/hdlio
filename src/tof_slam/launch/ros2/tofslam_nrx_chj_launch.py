"""
tofslam_nrx_chj_launch.py — Run TofSLAM on NRX nrx_chj dataset.

Sensors : front/spot ToF PointCloud2 + wheel odometry (/odom)
Estimator: frontend_w (wheel-prior IEKF)

Dataset: nrx/nrx_chj (rest_20260326_085634_time_aligned_0.db3)
Duration: ~69s

Log file: /root/ros2_ws/dump/nrx_chj_slam.log  (tofslam_node stdout+stderr)

Usage:
    ros2 launch tof_slam tofslam_nrx_chj_launch.py
    ros2 launch tof_slam tofslam_nrx_chj_launch.py rate:=0.5
    ros2 launch tof_slam tofslam_nrx_chj_launch.py log_path:=/path/to/custom.log
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, TimerAction, OpaqueFunction, LogInfo,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DUMP_DIR    = "/root/ros2_ws/dump"
DEFAULT_LOG = f"{DUMP_DIR}/nrx_chj_slam.log"


def launch_setup(context, *args, **kwargs):
    config   = LaunchConfiguration('config').perform(context)
    rate     = LaunchConfiguration('rate').perform(context)
    bag_path = LaunchConfiguration('bag_path').perform(context)
    log_path = LaunchConfiguration('log_path').perform(context)

    nodes = []

    # ------------------------------------------------------------------
    # 1. Static TF: base_link -> front_spot_pcl
    # ------------------------------------------------------------------
    nodes.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='front_spot_pcl_static_tf',
        arguments=[
            '0.169950',   # x
            '-0.000090',  # y
            '0.060050',   # z
            '-0.500040',  # qx
            '0.499920',   # qy
            '-0.500022',  # qz
            '0.500019',   # qw
            'base_link',       # parent frame
            'front_spot_pcl',  # child frame
        ],
        output='screen',
    ))

    # ------------------------------------------------------------------
    # 2. tofslam_node — stdout/stderr redirected to log_path
    # ------------------------------------------------------------------
    nodes.append(ExecuteProcess(
        cmd=[
            'bash', '-c',
            f'source /opt/ros/humble/setup.bash && '
            f'source /root/ros2_ws/install/setup.bash && '
            f'exec ros2 run tof_slam tofslam_node '
            f'--ros-args --params-file {config} '
            f'>> {log_path} 2>&1'
        ],
        output='screen',
        name='tofslam_node',
    ))

    # ------------------------------------------------------------------
    # 3. Bag playback (delayed 3 s to allow node startup)
    # ------------------------------------------------------------------
    # Bag play + 15s drain time: after bag finishes, sleep keeps the launch
    # alive so tofslam_node can drain its processing queue fully.
    nodes.append(TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    f'ros2 bag play {bag_path} '
                    f'--rate {rate} --clock --read-ahead-queue-size 1000 && '
                    f'sleep 15'
                ],
                output='screen',
            ),
        ],
    ))

    return nodes


def generate_launch_description():
    pkg_dir = get_package_share_directory('tof_slam')
    default_config = os.path.join(pkg_dir, 'config', 'nrx_chj.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='Path to YAML config file'),
        DeclareLaunchArgument(
            'bag_path',
            default_value='/root/dataset/nrx/nrx_chj',
            description='Path to the ROS 2 bag directory'),
        DeclareLaunchArgument(
            'rate',
            default_value='1.0',
            description='Bag playback rate multiplier'),
        DeclareLaunchArgument(
            'log_path',
            default_value=DEFAULT_LOG,
            description='File path for tofslam_node log output'),

        LogInfo(msg=[
            '[TofSLAM] nrx_chj  '
            f'log → {DEFAULT_LOG}'
        ]),
        OpaqueFunction(function=launch_setup),
    ])
