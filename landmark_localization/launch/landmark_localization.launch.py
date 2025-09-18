from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 声明启动参数
    pbstream_file_arg = DeclareLaunchArgument(
        'pbstream_file',
        default_value='/path/to/your/map.pbstream',
        description='Path to the pbstream file'
    )

    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan/front',
        description='Topic for the laser scan data'
    )

    matching_threshold_arg = DeclareLaunchArgument(
        'matching_threshold',
        default_value='0.2',
        description='Threshold for matching landmarks, unit: meter'
    )

    tf_time_tolerance_arg = DeclareLaunchArgument(
        'tf_time_tolerance',
        default_value='0.05',
        description='Time tolerance for TF, unit: second'
    )

    lidar_frame_arg = DeclareLaunchArgument(
        'lidar_frame',
        default_value='laser',
        description='Frame ID for the LiDAR, e.g., laser, two_d_lidar'
    )

    intensity_threshold_use_arg = DeclareLaunchArgument(
        'intensity_threshold_use',
        default_value='200',
        description='强度阈值'
    )
    
    # Get the package share directory
    pkg_share = get_package_share_directory('landmark_localization')
    
    # Create node configuration
    landmark_localization_node = Node(
        package='landmark_localization',
        executable='landmark_localization_node',
        name='landmark_localization_node',
        output='screen',
        parameters=[{
            'pbstream_file': LaunchConfiguration('pbstream_file'), 
            'lidar_frame': LaunchConfiguration('lidar_frame'), 
            'map_frame': 'map',
            'matching_threshold': LaunchConfiguration('matching_threshold'), 
            'scan_topic': LaunchConfiguration('scan_topic'), # sensor_msgs/msg/LaserScan
            'landmark_topic': '/landmark', # cartographer_ros_msgs/msg/LandmarkList
            'publish_visualization': True, # Whether to publish visualization markers
            'visualization_topic': '/landmark_localization_markers', # visualization_msgs/msg/MarkerArray
            'intensity_threshold_use': LaunchConfiguration('intensity_threshold_use'),
            'tf_time_tolerance': LaunchConfiguration('tf_time_tolerance'), 
        }]
    )
    
    return LaunchDescription([
        pbstream_file_arg,
        scan_topic_arg,
        matching_threshold_arg,
        tf_time_tolerance_arg,
        lidar_frame_arg,
        intensity_threshold_use_arg,
        landmark_localization_node
    ])