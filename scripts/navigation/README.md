# 3D Navigation Stack (A* + OctoMap)

This stack provides autonomous navigation for the drone in Gazebo.

## How to Run
1. Start the simulation stack (with `obstacle_maze.sdf`).
2. Run the navigation script:
   ```bash
   ./scripts/navigation/run_navigation.sh
   ```
3. Send a goal:
   ```bash
   python3 scripts/navigation/send_goal.py 5.0 2.0 1.5
   ```

## Components
- **Planner**: A* algorithm discretized at 0.2m.
- **Follower**: Sends setpoints to MAVROS.
- **SLAM**: RTAB-Map for visual odom and OctoMap.

## Communication Gate

`navigation_launch.py` starts `communication_preflight.py` by default. It
publishes `/navigation/preflight_ok=true` only after validating the expected
ROS message types, publisher/subscriber counts, QoS compatibility, live
rates, timestamps, TF (`odom` to the camera), and a bounded non-empty
OctoMap. `path_follower.py` refuses to configure or arm until that latch is
true. Set `require_preflight:=false` only for isolated debugging.

For the obstacle maze, use the safe takeoff point at the clear origin rather
than the facade point inside the maze wall:

```bash
ros2 launch /home/uas/scripts/navigation/navigation_launch.py \
  has_depth:=true enable_coverage_planner:=false \
  takeoff_x:=0.0 takeoff_y:=0.0 takeoff_z:=2.0
```

The depth sanitizer preserves Gazebo's sensor frame. The TF bridge publishes
the matching `x500_depth_0/rgbd_camera_link/rgbd_camera` transform; clouds
must not be relabelled without transforming their coordinates.
