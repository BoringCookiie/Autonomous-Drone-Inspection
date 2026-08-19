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
    ]

    # ------------------------------------------------------------------
    # Static nodes (always launched)
    # ------------------------------------------------------------------
    static_nodes = [
        Node(
            executable='python3', arguments=[os.path.join(nav_dir, 'planner_3d.py')],
            name='planner_3d', output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            executable='python3', arguments=[os.path.join(nav_dir, 'path_follower.py')],
            name='path_follower', output='screen',
            parameters=[{'use_sim_time': True}]
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

        if LaunchConfiguration('enable_slam').perform(context) == 'true':
            depth_available = LaunchConfiguration('has_depth').perform(context) == 'true'

            if depth_available:
                # ---- RGBD mode (gz_x500_depth) ----------------------------
                # RTAB-Map subscribes to both RGB and depth streams.
                rtabmap_params = [params_path]
                rtabmap_remaps = [
                    ('rgb/image',       '/camera/color/image_raw'),
                    ('depth/image',     '/camera/depth/image_raw'),
                    ('rgb/camera_info', '/camera/color/camera_info'),
                    ('odom',            '/uas1/local_position/odom'),
                ]
                # OctoMap is only useful when /points is bridged (depth model)
                nodes.append(Node(
                    package='octomap_server',
                    executable='octomap_server_node',
                    name='octomap_server',
                    output='screen',
                    parameters=[{'resolution': 0.2, 'frame_id': 'map'}],
                    remappings=[('cloud_in', '/points')]
                ))
            else:
                # ---- RGB-only mode (gz_x500_mono_cam) ---------------------
                # RTAB-Map in monocular/odometry-only mode — depth disabled.
                # subscribe_depth=false: don't wait for /camera/depth/image_raw
                # Mem/IncrementalMemory still builds a visual odometry map from RGB.
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
                # OctoMap skipped — no pointcloud without depth sensor

            nodes.append(Node(
                package='rtabmap_slam', executable='rtabmap', name='rtabmap',
                output='screen',
                parameters=rtabmap_params,
                remappings=rtabmap_remaps,
            ))

        return nodes

    return LaunchDescription(launch_args + [OpaqueFunction(function=compose_launch)])
