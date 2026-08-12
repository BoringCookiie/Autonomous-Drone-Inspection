FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1

# Install system & ROS2 build dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-gazebo-ros-pkgs \
    git \
    wget \
    curl \
    tmux \
    libgl1-mesa-glx \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Install PyTorch and AI dependencies
RUN pip3 install --no-cache-dir \
    torch torchvision --index-url https://download.pytorch.org/whl/cpu \
    ultralytics \
    git+https://github.com/openai/CLIP.git \
    opencv-python \
    pyyaml \
    matplotlib \
    pandas \
    scipy

WORKDIR /workspace
CMD ["/bin/bash"]
