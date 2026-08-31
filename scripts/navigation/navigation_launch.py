import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav_dir = "/home/uas/scripts/navigation"
    params_path = os.path.join(nav_dir, "rtabmap_params.yaml")

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    launch_args = [
        DeclareLaunchArgument(
            'enable_coverage_planner',
            default_value='true',
            description='Launch the boustrophedon coverage path planner node'
        ),
        DeclareLaunchArgument(
            'enable_slam',
            default_value='true',
            description='Launch RTAB-Map SLAM and OctoMap nodes'
        ),
        # has_depth: false = gz_x500_mono_cam (RGB only, no /camera/depth topic)
        #            true  = gz_x500_depth   (RGB+Depth, /points bridged)
        # Default is false to match the validated default model.
        DeclareLaunchArgument(
            'has_depth',
            default_value='false',
            description='Whether the drone model has a depth camera (gz_x500_depth=true, mono_cam=false)'
        ),
        DeclareLaunchArgument(
            'require_preflight',
            default_value='true',
            description='Do not arm until communication_preflight publishes readiness'
        ),
        DeclareLaunchArgument('takeoff_x', default_value='0.0'),
        DeclareLaunchArgument('takeoff_y', default_value='2.25'),
        DeclareLaunchArgument('takeoff_z', default_value='2.0'),
        DeclareLaunchArgument(
            'queue_planned_paths', default_value='true',
            description='Queue A* paths during coverage; false replaces paths for dynamic maze replanning'
        ),
    ]

    # ------------------------------------------------------------------
    # Static nodes (always launched)
    # ------------------------------------------------------------------
    static_nodes = [
        Node(
            executable='python3', arguments=[os.path.join(nav_dir, 'tf_bridge_node.py')],
            name='tf_bridge', output='screen',
            parameters=[{
                'use_sim_time': True,
                'camera_frame': 'depth_camera',
                'sensor_frame': 'x500_depth_0/rgbd_camera_link/rgbd_camera',
            }]
        ),
        Node(
            executable='python3', arguments=[os.path.join(nav_dir, 'planner_3d.py')],
            name='planner_3d', output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            executable='python3', arguments=[os.path.join(nav_dir, 'path_follower.py')],
            name='path_follower', output='screen',
            parameters=[{
                'use_sim_time': True,
                'require_preflight': LaunchConfiguration('require_preflight'),
                'takeoff_x': LaunchConfiguration('takeoff_x'),
                'takeoff_y': LaunchConfiguration('takeoff_y'),
                'takeoff_z': LaunchConfiguration('takeoff_z'),
                'queue_planned_paths': LaunchConfiguration('queue_planned_paths'),
            }]
        ),
    ]

    # ------------------------------------------------------------------
    # Coverage planner node (Phase 1 - boustrophedon lawnmower)
    # ------------------------------------------------------------------
    coverage_planner_node = Node(
        executable='python3',
        arguments=[os.path.join(nav_dir, 'coverage_planner.py')],
        name='coverage_planner',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            # Facade geometry matches earthen_heritage_wall.sdf:
            #   wall pose (0, 5, 2), size 10 x 0.5 x 4
            'facade_x_min':   -4.5,
            'facade_x_max':    4.5,
            'alt_min':         1.0,
            'alt_max':         3.5,
            'standoff_d':      2.5,
            'wall_y':          5.0,
            'overlap_ratio':   0.3,
        }]
    )

    def compose_launch(context, *args, **kwargs):
        nodes = list(static_nodes)

        if LaunchConfiguration('enable_coverage_planner').perform(context) == 'true':
            nodes.append(coverage_planner_node)

        depth_available = LaunchConfiguration('has_depth').perform(context) == 'true'

        if depth_available:
            # ---- Obstacle mapping (independent of SLAM) ----------------
            # The sanitizer must run before octomap_server: the gz bridge
            # marks clouds is_dense=true while many pixels are NaN, and
            # octomap silently drops every such insertion.
            nodes.append(Node(
                executable='python3',
                arguments=[os.path.join(nav_dir, 'depth_cloud_sanitizer.py')],
                name='depth_cloud_sanitizer', output='screen',
                parameters=[{'use_sim_time': True, 'output_frame_id': ''}]
            ))
            nodes.append(Node(
                package='octomap_server',
                executable='octomap_server_node',
                name='octomap_server',
                output='screen',
                parameters=[{'use_sim_time': True, 'resolution': 0.2, 'frame_id': 'odom'}],
                remappings=[('cloud_in', '/points_clean')]
            ))

        nodes.append(Node(
            executable='python3',
            arguments=[os.path.join(nav_dir, 'communication_preflight.py')],
            name='communication_preflight', output='screen',
            parameters=[{
                'use_sim_time': True,
                'require_depth': depth_available,
            }]
        ))

        if LaunchConfiguration('enable_slam').perform(context) == 'true':
            if depth_available:
                # ---- RGBD mode (gz_x500_depth) ----------------------------
                rtabmap_params = [params_path]
                rtabmap_remaps = [
                    ('rgb/image',       '/camera/color/image_raw'),
                    ('depth/image',     '/camera/depth/image_raw'),
                    ('rgb/camera_info', '/camera/color/camera_info'),
                    ('odom',            '/uas1/local_position/odom'),
                ]
            else:
                # ---- RGB-only mode (gz_x500_mono_cam) ---------------------
                rtabmap_params = [
                    params_path,
                    {'subscribe_depth': False,
                     'subscribe_rgb': True,
                     'approx_sync': True,
                     'queue_size': 10}
                ]
                rtabmap_remaps = [
                    ('rgb/image',       '/camera/color/image_raw'),
                    ('rgb/camera_info', '/camera/color/camera_info'),
                    ('odom',            '/uas1/local_position/odom'),
                ]

            nodes.append(Node(
                package='rtabmap_slam', executable='rtabmap', name='rtabmap',
                output='screen',
                parameters=rtabmap_params,
                remappings=rtabmap_remaps,
            ))

        return nodes

    return LaunchDescription(launch_args + [OpaqueFunction(function=compose_launch)])
