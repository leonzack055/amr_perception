from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # 定义节点
    reflector_detector_circle = Node(
        package='amr_reflective_circle',
        executable='reflector_detector_circle',
        name='reflector_detector_circle',
        output='screen'
    )

    return LaunchDescription([
        reflector_detector_circle
    ])
