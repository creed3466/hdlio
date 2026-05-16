"""
tofslam_m3dgr_launch.py — Run TofSLAM on any M3DGR Mid-360 sequence.

Usage:
    ros2 launch tof_slam tofslam_m3dgr_launch.py seq:=Dark01
    ros2 launch tof_slam tofslam_m3dgr_launch.py seq:=Dynamic03 rate:=1.0
    ros2 launch tof_slam tofslam_m3dgr_launch.py seq:=Occlusion03 config:=/path/to/custom.yaml
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
    seq      = LaunchConfiguration('seq').perform(context)
    bag_base = LaunchConfiguration('bag_base').perform(context)
    rate     = LaunchConfiguration('rate').perform(context)

    bag_path = os.path.join(bag_base, seq)

    # QoS override for bag replay (reliable → best_effort)
    qos_override = '/tmp/qos_override.yaml'

    nodes = []

    # tofslam_node
    nodes.append(Node(
        package='tof_slam',
        executable='tofslam_node',
        name='tofslam_node',
        output='screen',
        parameters=[
            config,
            {'use_sim_time': True},
        ],
    ))

    # Bag playback (delayed 3s for node startup)
    nodes.append(TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg=[f'Playing bag: {bag_path} at rate={rate}']),
            ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'play', bag_path,
                    '--rate', rate,
                    '--clock',
                    '--read-ahead-queue-size', '1000',
                    '--qos-profile-overrides-path', qos_override,
                ],
                output='screen',
            ),
        ],
    ))

    return nodes


def generate_launch_description():
    pkg_dir = get_package_share_directory('tof_slam')
    default_config = os.path.join(
        pkg_dir, 'config', 'm3dgr_mid360.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config', default_value=default_config,
            description='Path to YAML configuration file'),
        DeclareLaunchArgument(
            'seq', default_value='Dark01',
            description='M3DGR sequence name (Dark01, Dark02, Dynamic03, Dynamic04, '
                        'Occlusion03, Occlusion04, Varying-illu03, Varying-illu04, Varying-illu05)'),
        DeclareLaunchArgument(
            'bag_base', default_value='/root/data/m3dgr',
            description='Base path to M3DGR bags'),
        DeclareLaunchArgument(
            'rate', default_value='1.0',
            description='Bag playback rate multiplier'),

        LogInfo(msg=['TofSLAM M3DGR Mid-360 launch']),
        OpaqueFunction(function=launch_setup),
    ])
