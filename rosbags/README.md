# ROS2 Bags Directory

Store flight test recordings and inspection telemetry logs (`*.db3`, `*.mcap`).
Generated during simulated or real UAV runs and analyzed using `scripts/analyze_rosbags.py`.

The `legacy/` subtree contains rosbag evidence migrated from the original `droneit` project.

The flight launcher records the canonical RGB stream at `/camera/color/image_raw` and
exports it as `analysis/camera_color_image_raw.mp4`. Depth is recorded only when the
depth model is selected or `FLY_PATTERN_RECORD_DEPTH=1` is set. A video export is
considered invalid when the vehicle moved but every decoded RGB frame is identical.
