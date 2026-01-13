#!/usr/bin/env python3
"""
EuRoC Dataset Evaluation Launch File
Run SLAM on EuRoC MAV sequences for benchmarking

Author: Al Numan
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Launch arguments
    sequence_arg = DeclareLaunchArgument(
        'sequence',
        default_value='MH_01_easy',
        description='EuRoC sequence name'
    )

    dataset_path_arg = DeclareLaunchArgument(
        'dataset_path',
        default_value='/data/euroc',
        description='Path to EuRoC dataset'
    )

    playback_rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='1.0',
        description='Rosbag playback rate'
    )

    save_trajectory_arg = DeclareLaunchArgument(
        'save_trajectory',
        default_value='true',
        description='Save estimated trajectory'
    )

    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='/tmp/slam_results',
        description='Output directory for results'
    )

    def launch_setup(context):
        nodes = []

        sequence = LaunchConfiguration('sequence').perform(context)
        dataset_path = LaunchConfiguration('dataset_path').perform(context)
        rate = LaunchConfiguration('rate').perform(context)
        save_traj = LaunchConfiguration('save_trajectory').perform(context)
        output_dir = LaunchConfiguration('output_dir').perform(context)

        pkg_path = get_package_share_directory('visual_slam')

        # Config files
        slam_config = os.path.join(pkg_path, 'config', 'slam_config.yaml')
        camera_config = os.path.join(pkg_path, 'config', 'camera', 'euroc.yaml')
        rviz_config = os.path.join(pkg_path, 'rviz', 'slam.rviz')

        # Bag file path
        bag_path = os.path.join(dataset_path, sequence, f'{sequence}.bag')

        # SLAM node
        slam_node = Node(
            package='visual_slam',
            executable='slam_node',
            name='visual_slam',
            output='screen',
            parameters=[
                slam_config,
                {'camera_config': camera_config},
                {'mode': 'stereo_imu'},
                {'use_imu': True},
                {'save_trajectory': save_traj == 'true'},
                {'trajectory_path': os.path.join(output_dir, f'{sequence}_trajectory.csv')},
            ],
            remappings=[
                ('left/image_raw', '/cam0/image_raw'),
                ('right/image_raw', '/cam1/image_raw'),
                ('imu', '/imu0'),
            ]
        )
        nodes.append(slam_node)

        # RViz
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
        nodes.append(rviz_node)

        # Rosbag play (using ROS2 bag if converted, otherwise ros1_bridge needed)
        # This assumes the bag has been converted to ROS2 format
        rosbag_play = ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_path, '--rate', rate],
            output='screen'
        )
        nodes.append(rosbag_play)

        return nodes

    return LaunchDescription([
        sequence_arg,
        dataset_path_arg,
        playback_rate_arg,
        save_trajectory_arg,
        output_dir_arg,
        OpaqueFunction(function=launch_setup)
    ])
