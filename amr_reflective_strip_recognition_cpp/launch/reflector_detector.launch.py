from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # 定义节点
    reflector_detector_node = Node(
        package='amr_reflective_strip_recognition_cpp',
        executable='reflector_detector_node',
        name='reflector_detector_node',
        output='screen'
    )

    return LaunchDescription([
        reflector_detector_node
    ])
