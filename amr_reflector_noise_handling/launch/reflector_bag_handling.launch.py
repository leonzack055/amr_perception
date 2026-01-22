#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('amr_reflector_noise_handling')
    config_file = os.path.join(pkg_dir, 'config', 'reflector_noise_params.yaml')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'bag_path',
            default_value='/workspaces/ros-dev/amr_ws/bag/linesCargo.bag',
            description='建图数据包路径'
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='Laser scan topic'
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odom_combined',
            description='Wheel Odometry Topic in the bag'
        ),
        DeclareLaunchArgument(
            'landmark_topic',
            default_value='/landmark_noise',
            description='Landmark output topic'
        ),
        DeclareLaunchArgument(
            'marker_topic',
            default_value='/reflector_noise_markers',
            description='Visualization marker topic'
        ),
        DeclareLaunchArgument(
            'compensated_cloud_topic',
            default_value='/compensated_cloud',
            description='Compensated point cloud topic for visualization'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=config_file,
            description='Path to parameters file'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level (debug, info, warn, error, fatal)'
        ),
        
        # Reflector noise handling node
        Node(
            package='amr_reflector_noise_handling',
            executable='reflector_noise_bag_node',
            name='reflector_noise_bag_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'bag_path': LaunchConfiguration('bag_path')},
                {'log_level': LaunchConfiguration('log_level')},
            ],
            arguments=["--ros-args", "--log-level", LaunchConfiguration('log_level')],
            remappings=[
            ]
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()