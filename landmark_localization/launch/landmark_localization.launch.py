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

    map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value='map',
        description='Map frame ID'
    )

    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Base frame ID'
    )

    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame ID'
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

    intensity_threshold_use_arg = DeclareLaunchArgument(
        'intensity_threshold_use',
        default_value='1600',
        description='强度阈值'
    )

    min_landmarks_for_pose_arg = DeclareLaunchArgument(
        'min_landmarks_for_pose',
        default_value='3',
        description='Minimum number of matched landmarks required to compute robot pose'
    )
    
    # 激光雷达1参数
    use_lidar1_arg = DeclareLaunchArgument(
        'use_lidar1',
        default_value='true',
        description='Whether to use lidar1'
    )
    
    scan1_topic_arg = DeclareLaunchArgument(
        'scan1_topic',
        default_value='/scan',
        description='Topic for lidar1 scan data'
    )
    
    # 激光雷达2参数
    use_lidar2_arg = DeclareLaunchArgument(
        'use_lidar2',
        default_value='true',
        description='Whether to use lidar2'
    )
    
    scan2_topic_arg = DeclareLaunchArgument(
        'scan2_topic',
        default_value='/rear_scan',
        description='Topic for lidar2 scan data'
    )

    use_combine_arg = DeclareLaunchArgument(
        'use_combine',
        default_value='true',
        description='Whether to use combined processing for lidar1 and lidar2'
    )

    use_calculate_filter_arg = DeclareLaunchArgument(
        'use_calculate_filter',
        default_value='true',
        description='Whether to use filtering for pose calculation'
    )

    filter_num_arg = DeclareLaunchArgument(
        'filter_num',
        default_value='10',
        description='Filter number: consecutive times required for same landmark combination'
    )

    # Get the package share directory
    pkg_share = get_package_share_directory('landmark_localization')
    
    # Create node configuration
    landmark_localization_node = Node(
        package='landmark_localization',
        executable='landmark_localization_node',
        name='landmark_localization_node',
        output={'stdout': 'log', 'stderr': 'log'},
        parameters=[{
            'pbstream_file': LaunchConfiguration('pbstream_file'),
            'map_frame': LaunchConfiguration('map_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'matching_threshold': LaunchConfiguration('matching_threshold'),
            'publish_visualization': True,
            'intensity_threshold_use': LaunchConfiguration('intensity_threshold_use'),
            'tf_time_tolerance': LaunchConfiguration('tf_time_tolerance'),
            'min_landmarks_for_pose': LaunchConfiguration('min_landmarks_for_pose'),
            'use_calculate_filter': LaunchConfiguration('use_calculate_filter'),
            'filter_num': LaunchConfiguration('filter_num'),
            
            # 激光雷达1参数
            'use_lidar1': LaunchConfiguration('use_lidar1'),
            'scan1_topic': LaunchConfiguration('scan1_topic'),
            
            # 激光雷达2参数
            'use_lidar2': LaunchConfiguration('use_lidar2'),
            'scan2_topic': LaunchConfiguration('scan2_topic'),

            'use_combine': LaunchConfiguration('use_combine'),
            
            # 固定参数
            'landmark_topic': '/landmark',
            'visualization_topic': '/landmark_localization_markers',
            'landmark_localization_topic': '/global_pose_qr',
            'initial_pose_topic': '/initial_pose',
        }]
    )
    
    return LaunchDescription([
        pbstream_file_arg,
        map_frame_arg,
        base_frame_arg,
        odom_frame_arg,
        matching_threshold_arg,
        tf_time_tolerance_arg,
        intensity_threshold_use_arg,
        min_landmarks_for_pose_arg,
        use_lidar1_arg,
        scan1_topic_arg,
        use_lidar2_arg,
        scan2_topic_arg,
        use_combine_arg,
        use_calculate_filter_arg,
        filter_num_arg,
        landmark_localization_node
    ])