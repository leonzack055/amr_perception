from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取配置文件路径
    config_dir = os.path.join(
        get_package_share_directory('amr_reflective_strip_recognition'),
        'config',
        'reflective_strip_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='amr_reflective_strip_recognition',
            executable='circle_detector',
            parameters=[config_dir]  # 直接传入YAML文件路径
        )
    ])
