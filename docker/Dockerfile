FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1
ENV YOLO_CONFIG_DIR=/tmp

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

# Keep the Python build toolchain mutually compatible before installing packages
# that still generate metadata through setup.py (notably OpenAI CLIP).
RUN python3 -m pip install --no-cache-dir --upgrade \
    pip "setuptools>=77,<80" wheel "packaging>=24"

# Install CPU PyTorch from its dedicated index first. Keeping this command separate
# prevents pip from looking for PyPI-only packages (for example ultralytics) there.
RUN python3 -m pip install --no-cache-dir \
    --ignore-installed \
    --index-url https://download.pytorch.org/whl/cpu \
    torch torchvision

# Install the remaining AI dependencies from the default PyPI index.
RUN python3 -m pip install --no-cache-dir \
    "numpy<2" \
    "opencv-python<4.12" \
    ultralytics \
    git+https://github.com/openai/CLIP.git \
    pyyaml \
    matplotlib \
    pandas \
    scipy

WORKDIR /workspace
CMD ["/bin/bash"]
