# Migration Notes

The original `droneit` project is no longer a separate runtime dependency. Its operational assets were moved into this repository:

| Original location | New location |
|---|---|
| `scripts/fly_pattern.py` | `scripts/simulation/fly_pattern.py` |
| `scripts/gz_camera_ros_bridge.py` | `scripts/simulation/gz_camera_ros_bridge.py` |
| `scripts/Navigation/` | `scripts/navigation/` |
| `scripts/analyze_rosbags.py` | `scripts/analysis/analyze_rosbags.py` |
| `shared/gz_worlds/` | `gazebo_simulation/worlds/` |
| `shared/rosbags/` | `rosbags/legacy/` |
| `docker/sim/Dockerfile` | `docker/Dockerfile` |
| `docker/sim/entrypoint.sh` | `docker/entrypoint.sh` |
| `docker/ai/` | `docker/ai/` |

The migrated launcher keeps the old `/uas1` MAVROS namespace and adds the inspection package on top. The camera bridge now aliases Gazebo image streams to the stable inspection topics:

- `/camera/color/image_raw`
- `/camera/depth/image_raw`
- `/camera/color/camera_info`

The new `waypoint_reached_node` converts streamed MAVROS setpoints and local pose into `/uav/waypoint_reached`. The capture node additionally republishes the depth frame frozen at the waypoint moment on `/inspection/captured_depth`, so revisit unprojection uses capture-time geometry. The migrated A* planner consumes `/planner/revisit_waypoints` in addition to its original manual `/navigation/goal` input.

The original files are retained as historical documentation under `docs/legacy/`. The old project directory itself is intentionally untouched.
