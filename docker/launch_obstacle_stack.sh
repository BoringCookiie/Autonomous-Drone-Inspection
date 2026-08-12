#!/bin/bash
# Launch script for Gazebo earthen heritage world and obstacle avoidance stack
echo "[INFO] Launching Gazebo Earthen Heritage Wall World..."
ros2 launch gazebo_ros gazebo.launch.py world:=/workspace/Autonomous-Drone-Inspection/gazebo_simulation/worlds/earthen_heritage_wall.world
