from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # 声明launch参数
    intensity_threshold_use_arg = DeclareLaunchArgument(
        'intensity_threshold_use',
        default_value='200',
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
        default_value='12',
        description='聚类最小点数'
    )
    diameter_min_arg = DeclareLaunchArgument(
        'diameter_min',
        default_value='0.05',
        description='最小估计直径'
    )
    
    diameter_max_arg = DeclareLaunchArgument(
        'diameter_max',
        default_value='0.15',
        description='最大估计直径'
    )

    bar_radius_arg = DeclareLaunchArgument(
        'residual_real',
        default_value='0.025',
        description='反光柱半径'
    )

    arc_min_points_arg = DeclareLaunchArgument(
        'arc_min_points',
        default_value='10',
        description='拟合圆弧最小点数'
    )

    rotation_weight_arg = DeclareLaunchArgument(
            'landmark_rotation_weight',
            default_value='1e2',
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
        executable='reflector_detector_circle_fit',
        name='reflector_detector_circle_fit',
        output='screen',
        arguments=['--ros-args', '--log-level', 'WARN'],
        parameters=[{
            'use_sim_time': True,
            'intensity_threshold_use': LaunchConfiguration('intensity_threshold_use'),
            'arc_threshold': LaunchConfiguration('arc_threshold'),
            'cluster_eps': LaunchConfiguration('cluster_eps'),
            'min_cluster_points': LaunchConfiguration('min_cluster_points'),
            'diameter_min': LaunchConfiguration('diameter_min'),
            'diameter_max': LaunchConfiguration('diameter_max'),
            'arc_min_points': LaunchConfiguration('arc_min_points'),
            'residual_real': LaunchConfiguration('residual_real'),
            "landmark_rotation_weight": LaunchConfiguration('landmark_rotation_weight'),
            "landmark_translation_weight": LaunchConfiguration('landmark_translation_weight'),
        }],
        remappings=[
            ('scan', '/scan/front'),
        ]
    )

    return LaunchDescription([
        # 参数声明
        intensity_threshold_use_arg,
        arc_threshold_arg,
        cluster_eps_arg,
        arc_min_points_arg,
        min_cluster_points_arg,
        diameter_min_arg,
        diameter_max_arg,
        bar_radius_arg,
        rotation_weight_arg,
        translation_weight_arg,
        # 节点
        reflector_detector_circle,
    ])
