from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory  # 导入此模块
import os

def generate_launch_description():

    # 获取当前包的路径
    package_share_directory = os.path.join(
        get_package_share_directory('amr_reflective_strip_recognition_cpp'), 'config', 'reflective_strip_params.yaml'
    )

    # 声明参数文件路径
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=package_share_directory,
        description='Path to the YAML configuration file'
    )

    # 定义节点
    reflector_detector_node = Node(
        package='amr_reflective_strip_recognition_cpp',
        executable='reflector_detector_node',
        parameters=[LaunchConfiguration('config_file')],
        name='reflector_detector_node',
        output='screen'
    )

    return LaunchDescription([
        config_file_arg,
        reflector_detector_node
    ])
