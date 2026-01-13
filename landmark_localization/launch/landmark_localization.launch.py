from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # 声明启动参数
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (debug, info, warn, error, fatal)'        
    )

    output_type_arg = DeclareLaunchArgument(
        'output_type',
        default_value='screen',
        description='Output type'        
    )

    # yaml配置文件路径
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            get_package_share_directory('tf_publisher_pkg'),
            'config',
            'landmark_localization_params.yaml'
        ),
        description='Path to the YAML configuration file'
    )

    # Get the package share directory
    pkg_share = get_package_share_directory('landmark_localization')
    
    # Create node configuration
    landmark_localization_node = Node(
        package='landmark_localization',
        executable='landmark_localization_node',
        name='landmark_localization_node',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        output=LaunchConfiguration('output_type'),
        parameters=[{
            'config_file': LaunchConfiguration('config_file')
        }]
    )
    
    return LaunchDescription([
        log_level_arg,
        output_type_arg,
        config_file_arg,
        landmark_localization_node
    ])