# Setup & Installation Guide

This document outlines environment initialization, Docker container creation, dependencies installation, and simulation setup.

## 1. Prerequisites
- **OS**: Ubuntu 22.04 LTS (Native or via Docker / WSL2)
- **ROS2 Version**: ROS2 Humble Hawksbill or Jazzy Jalisco
- **Python**: Python 3.10+
- **GPU Acceleration**: NVIDIA GPU with CUDA 11.8+ / 12.0+ (recommended for PyTorch, VLM & CLIP inference)

## 2. Docker Containerized Setup (Recommended)

To run the complete system including PX4 SITL and Gazebo simulation without host machine conflicts:

```bash
cd docker/
# Build container image
docker compose build sim_stack

# Run stack container
docker compose up -d sim_stack

# Attach shell to running container
docker exec -it earthen_uav_sim bash
```

Inside container:
```bash
./docker/bootstrap_px4.sh
./docker/launch_obstacle_stack.sh
```

## 3. Host System Native Workspace Installation

If installing natively:

### Dependencies Installation
```bash
# System packages
sudo apt update && sudo apt install -y \
    ros-humble-desktop \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    ros-humble-nav-msgs \
    python3-colcon-common-extensions \
    python3-pip

# Python AI dependencies
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118
pip install ultralytics git+https://github.com/openai/CLIP.git opencv-python pyyaml matplotlib pandas
```

### Build ROS2 Package
```bash
cd Autonomous-Drone-Inspection
colcon build --symlink-install --packages-select uas_earthen_inspection
source install/setup.bash
```
