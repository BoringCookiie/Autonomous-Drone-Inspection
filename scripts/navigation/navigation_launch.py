import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nav_dir = "/home/uas/scripts/navigation"
    params_path = os.path.join(nav_dir, "rtabmap_params.yaml")

    return LaunchDescription([
        Node(
            package='rtabmap_slam', executable='rtabmap', name='rtabmap',
            output='screen',
            parameters=[params_path],
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('depth/image', '/camera/depth/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('odom', '/uas1/local_position/odom')
            ]
        ),
        Node(
            package='octomap_server', executable='octomap_server_node', name='octomap_server',
            output='screen',
            parameters=[{'resolution': 0.2, 'frame_id': 'map'}],
            remappings=[('cloud_in', '/points')]
        ),
        Node(
            executable='python3', arguments=[os.path.join(nav_dir, "planner_3d.py")],
            name='planner_3d', output='screen'
        ),
        Node(
            executable='python3', arguments=[os.path.join(nav_dir, "path_follower.py")],
            name='path_follower', output='screen'
        ),
    ])
