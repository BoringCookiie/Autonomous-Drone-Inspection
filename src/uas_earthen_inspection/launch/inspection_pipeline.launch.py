import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('uas_earthen_inspection')
    default_config = os.path.join(pkg_dir, 'config', 'inspection_params.yaml')

    # Launch arguments for 3x2 Matrix Evaluation
    detector_backend_arg = DeclareLaunchArgument(
        'detector_backend',
        default_value='rag_vlm',
        description='Detector backend model: raw_vlm | rag_vlm | yolo'
    )

    flight_strategy_arg = DeclareLaunchArgument(
        'flight_strategy',
        default_value='revisit',
        description='Flight execution strategy: single_pass | revisit'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to ROS2 YAML parameters configuration file'
    )

    detector_backend = LaunchConfiguration('detector_backend')
    flight_strategy = LaunchConfiguration('flight_strategy')
    config_file = LaunchConfiguration('config_file')

    # 1. Per-Waypoint Capture Node
    per_waypoint_capture_node = Node(
        package='uas_earthen_inspection',
        executable='per_waypoint_capture_node',
        name='per_waypoint_capture_node',
        output='screen',
        parameters=[config_file]
    )

    # 2. Detection Node (Parameterized backend)
    detection_node = Node(
        package='uas_earthen_inspection',
        executable='detection_node',
        name='detection_node',
        output='screen',
        parameters=[
            config_file,
            {'detector_backend': detector_backend}
        ]
    )

    # 3. Revisit Waypoint Generator Node
    revisit_waypoint_generator_node = Node(
        package='uas_earthen_inspection',
        executable='revisit_waypoint_generator',
        name='revisit_waypoint_generator',
        output='screen',
        parameters=[
            config_file,
            {'flight_strategy': flight_strategy}
        ]
    )

    return LaunchDescription([
        detector_backend_arg,
        flight_strategy_arg,
        config_file_arg,
        per_waypoint_capture_node,
        detection_node,
        revisit_waypoint_generator_node
    ])
