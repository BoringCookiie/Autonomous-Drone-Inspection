#!/bin/bash
# Bootstrap script for PX4 SITL and Gazebo simulation stack setup
echo "[INFO] Bootstrapping PX4 Autopilot SITL Environment..."
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/workspace/Autonomous-Drone-Inspection/gazebo_simulation/models
echo "[INFO] Gazebo model path set to: $GAZEBO_MODEL_PATH"
echo "[INFO] PX4 SITL Bootstrap Complete."
