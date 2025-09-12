from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # 声明launch参数
    intensity_threshold_use_arg = DeclareLaunchArgument(
        'intensity_threshold_use',
        default_value='1500',
        description='强度阈值'
    )
    
    arc_threshold_arg = DeclareLaunchArgument(
        'arc_threshold',
        default_value='0.1',
        description='弧线阈值'
    )
    
    cluster_eps_arg = DeclareLaunchArgument(
        'cluster_eps',
        default_value='0.064',
        description='聚类的距离'
    )
    
    min_cluster_points_arg = DeclareLaunchArgument(
        'min_cluster_points',
        default_value='4',
        description='聚类最小点数'
    )
    percentage_min_arg = DeclareLaunchArgument(
        'percentage_min',
        default_value='0.2',
        description='强度最小占比'
    )
    
    percentage_max_arg = DeclareLaunchArgument(
        'percentage_max',
        default_value='0.8',
        description='强度最大占比'
    )

    bar_radius_arg = DeclareLaunchArgument(
        'residual_real',
        default_value='0.032',
        description='反光柱半径'
    )

    rotation_weight_arg = DeclareLaunchArgument(
            'landmark_rotation_weight',
            default_value='1e-2',
            description='反光柱的旋转权重'
        )

    translation_weight_arg = DeclareLaunchArgument(
            'landmark_translation_weight',
            default_value='1e5',
            description='反光柱的平移权重'
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
            'cluster_eps': LaunchConfiguration('cluster_eps'),
            'min_cluster_points': LaunchConfiguration('min_cluster_points'),
            'percentage_min': LaunchConfiguration('percentage_min'),
            'percentage_max': LaunchConfiguration('percentage_max'),
            'residual_real': LaunchConfiguration('residual_real'),
            "landmark_rotation_weight": LaunchConfiguration('landmark_rotation_weight'),
            "landmark_translation_weight": LaunchConfiguration('landmark_translation_weight'),
        }]
    )

    return LaunchDescription([
        # 参数声明
        intensity_threshold_use_arg,
        arc_threshold_arg,
        cluster_eps_arg,
        min_cluster_points_arg,
        percentage_min_arg,
        percentage_max_arg,
        bar_radius_arg,
        rotation_weight_arg,
        translation_weight_arg,
        # 节点
        reflector_detector_circle
    ])
