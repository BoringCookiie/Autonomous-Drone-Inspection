FROM ros:humble-ros-base

# 0. Prevent interactive prompts during build
ARG DEBIAN_FRONTEND=noninteractive

# /opt/ros/*/setup.bash uses bash-isms (arrays); RUN defaults to /bin/sh otherwise
SHELL ["/bin/bash", "-c"]

# 1. Base dependencies and Gazebo/PX4 build tooling
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-setuptools \
    python3-rosdep \
    python3-colcon-common-extensions \
    git \
    cmake \
    ninja-build \
    wget \
    lsb-release \
    gnupg \
    curl \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Ajout de la clé GPG et du dépôt officiel Gazebo (pour Garden/Sim8)
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# 2. Installation des dépendances Gazebo, Navigation et Mapping
RUN apt-get update && apt-get install -y \
    libgz-sim8-dev \
    gz-garden \
    python3-gz-msgs10 \
    python3-gz-transport13 \
    ros-humble-rtabmap-ros \
    ros-humble-octomap-server \
    ros-humble-octomap-ros \
    ros-humble-octomap-msgs \
    ros-humble-mavros \
    ros-humble-mavros-msgs \
    ros-humble-vision-msgs \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-storage-default-plugins \
    ros-humble-cv-bridge \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-image \
    ros-humble-rviz2 \
    && /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh \
    && rm -rf /var/lib/apt/lists/*

# 3. Setup Python dependencies for PX4
# These are the most common requirements to avoid long wait times during bootstrap
RUN pip3 install --no-cache-dir \
    kconfiglib \
    jinja2 \
    PyYAML \
    jsonschema \
    future \
    toml \
    numpy \
    packaging \
    empy==3.3.4 \
    pyros-genmsg \
    setuptools

RUN pip3 install --no-cache-dir --upgrade 'setuptools<80' packaging

# 4. Python dependencies for the inspection package and offline analysis
RUN pip3 install --no-cache-dir \
    torch torchvision --index-url https://download.pytorch.org/whl/cpu

RUN pip3 install --no-cache-dir \
    ultralytics \
    pyyaml \
    matplotlib \
    pandas \
    scipy

# ROS Humble cv_bridge is built against NumPy 1.x. Keep the Python ML stack
# compatible with that ABI and with the system OpenCV bindings.
RUN pip3 install --no-cache-dir --force-reinstall \
    'numpy<2' \
    'opencv-python==4.10.0.84'

# 5. Configuration de l'utilisateur
RUN useradd -m uas && echo "uas:uas" | chpasswd && adduser uas sudo
RUN echo "uas ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER uas
WORKDIR /home/uas
ENV PATH="${PATH}:/home/uas/.local/bin"

# Pre-create PX4 dir as uas so a fresh named volume inherits the ownership
# (Docker copies image dir ownership into new volumes on first mount).
RUN mkdir -p /home/uas/PX4-Autopilot
RUN mkdir -p /home/uas/ros2_ws && chown -R uas:uas /home/uas/ros2_ws

# Setup ROS entrypoint
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "[ -f /home/uas/ros2_ws/install/setup.bash ] && source /home/uas/ros2_ws/install/setup.bash" >> ~/.bashrc

# Entrypoint script
COPY docker/entrypoint.sh /entrypoint.sh
# Note: we can't chmod here as root because we switched to user uas.
# We'll switch back to root briefly to set it up.
USER root
RUN chmod +x /entrypoint.sh
USER uas

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
