# ============================================================
#  UAS AI Stack — Dockerfile
#  Base: osrf/ros:humble-desktop-full
#
#  Installs:
#    • ROS2 Humble bag/image tooling
#    • Python ML stack for YOLOv11 workflow
#    • Ultralytics for training + offline inference on rosbag frames
# ============================================================

FROM osrf/ros:humble-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=uas
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME \
 && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME \
 && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    git \
    wget \
    curl \
    python3-pip \
    python3-setuptools \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    python3-opencv \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-storage-default-plugins \
    ros-humble-rosbag2-compression \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-vision-msgs \
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-rviz2 \
    ros-humble-rqt-image-view \
 && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir --default-timeout=1000 \
    --upgrade pip \
    ultralytics \
    albumentations \
    matplotlib \
    pandas \
    pyyaml \
    scikit-learn \
    opencv-python-headless

RUN rosdep init 2>/dev/null || true \
 && rosdep update

RUN mkdir -p /home/$USERNAME/ros2_ws/src \
             /home/$USERNAME/rosbags \
             /home/$USERNAME/datasets \
             /home/$USERNAME/weights \
 && chown -R $USERNAME:$USERNAME /home/$USERNAME

COPY docker/ai/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER $USERNAME
RUN echo "" >> ~/.bashrc \
 && echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
 && echo "[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash" >> ~/.bashrc \
 && echo "export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-42}" >> ~/.bashrc \
 && echo "PS1='\\[\\033[01;36m\\][AI|D\$ROS_DOMAIN_ID]\\[\\033[00m\\] \\[\\033[01;34m\\]\\w\\[\\033[00m\\]\\$ '" >> ~/.bashrc

WORKDIR /home/$USERNAME

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
