#!/usr/bin/env python3
"""
Visual SLAM Launch File
Configurable launch for stereo/stereo-inertial SLAM

Author: Al Numan
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import yaml


def generate_launch_description():
    # Package share directory
    pkg_share = FindPackageShare('visual_slam')

    # Launch arguments
    camera_arg = DeclareLaunchArgument(
        'camera',
        default_value='realsense',
        description='Camera type: realsense, zed, custom'
    )

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='stereo_imu',
        description='SLAM mode: stereo, stereo_imu, rgbd'
    )

    config_arg = DeclareLaunchArgument(
        'config',
        default_value='',
        description='Path to custom config file (optional)'
    )

    vocabulary_arg = DeclareLaunchArgument(
        'vocabulary',
        default_value='',
        description='Path to ORB vocabulary file'
    )

    # Topic remapping arguments
    left_image_arg = DeclareLaunchArgument(
        'left_image',
        default_value='',
        description='Left image topic (overrides camera config)'
    )

    right_image_arg = DeclareLaunchArgument(
        'right_image',
        default_value='',
        description='Right image topic (overrides camera config)'
    )

    imu_arg = DeclareLaunchArgument(
        'imu',
        default_value='',
        description='IMU topic (overrides camera config)'
    )

    # Visualization
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz visualization'
    )

    def launch_setup(context):
        nodes = []

        camera = LaunchConfiguration('camera').perform(context)
        mode = LaunchConfiguration('mode').perform(context)
        config_file = LaunchConfiguration('config').perform(context)
        left_image = LaunchConfiguration('left_image').perform(context)
        right_image = LaunchConfiguration('right_image').perform(context)
        imu_topic = LaunchConfiguration('imu').perform(context)
        launch_rviz = LaunchConfiguration('rviz').perform(context)

        # Determine config paths
        pkg_path = get_package_share_directory('visual_slam')

        if not config_file:
            config_file = os.path.join(pkg_path, 'config', 'slam_config.yaml')

        camera_config_file = os.path.join(
            pkg_path, 'config', 'camera',
            f'{camera}_d435i.yaml' if camera == 'realsense' else f'{camera}.yaml'
        )

        # Load camera config for default topics
        camera_topics = {}
        if os.path.exists(camera_config_file):
            with open(camera_config_file, 'r') as f:
                camera_cfg = yaml.safe_load(f)
                camera_topics = camera_cfg.get('topics', {})

        # Build topic remappings
        remappings = []
        if left_image:
            remappings.append(('left/image_raw', left_image))
        elif 'left_image' in camera_topics:
            remappings.append(('left/image_raw', camera_topics['left_image']))

        if right_image:
            remappings.append(('right/image_raw', right_image))
        elif 'right_image' in camera_topics:
            remappings.append(('right/image_raw', camera_topics['right_image']))

        if imu_topic:
            remappings.append(('imu', imu_topic))
        elif 'imu' in camera_topics:
            remappings.append(('imu', camera_topics['imu']))

        # SLAM node
        slam_node = Node(
            package='visual_slam',
            executable='slam_node',
            name='visual_slam',
            output='screen',
            parameters=[
                config_file,
                {'camera_config': camera_config_file},
                {'mode': mode},
                {'use_imu': mode == 'stereo_imu'},
            ],
            remappings=remappings
        )
        nodes.append(slam_node)

        # RViz
        if launch_rviz.lower() == 'true':
            rviz_config = os.path.join(pkg_path, 'rviz', 'slam.rviz')
            rviz_node = Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                output='screen'
            )
            nodes.append(rviz_node)

        return nodes

    return LaunchDescription([
        camera_arg,
        mode_arg,
        config_arg,
        vocabulary_arg,
        left_image_arg,
        right_image_arg,
        imu_arg,
        rviz_arg,
        OpaqueFunction(function=launch_setup)
    ])
