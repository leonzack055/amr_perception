from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # 声明launch参数
    intensity_threshold_use_arg = DeclareLaunchArgument(
        'intensity_threshold_use',
        default_value='1000',
        description='强度阈值'
    )
    
    arc_threshold_arg = DeclareLaunchArgument(
        'arc_threshold',
        default_value='0.1',
        description='弧线阈值'
    )
    
    max_arc_feature_arg = DeclareLaunchArgument(
        'max_arc_feature',
        default_value='30.0',
        description='最大弧线特征'
    )
    cluster_eps_arg = DeclareLaunchArgument(
        'cluster_eps',
        default_value='0.064',
        description='聚类的距离'
    )
    
    min_cluster_points_arg = DeclareLaunchArgument(
        'min_cluster_points',
        default_value='5.0',
        description='聚类最小点数'
    )
    # 定义节点
    reflector_detector_circle = Node(
        package='amr_reflective_circle',
        executable='reflector_detector_circle',
        name='reflector_detector_circle',
        output='screen',
        parameters=[{
            'intensity_threshold_use': LaunchConfiguration('intensity_threshold_use'),
            'arc_threshold': LaunchConfiguration('arc_threshold'),
            'max_arc_feature': LaunchConfiguration('max_arc_feature'),
            'cluster_eps': LaunchConfiguration('cluster_eps'),
            'min_cluster_points': LaunchConfiguration('min_cluster_points'),
        }]
    )

    return LaunchDescription([
        # 参数声明
        intensity_threshold_use_arg,
        arc_threshold_arg,
        max_arc_feature_arg,
        cluster_eps_arg,
        min_cluster_points_arg,
        # 节点
        reflector_detector_circle
    ])
