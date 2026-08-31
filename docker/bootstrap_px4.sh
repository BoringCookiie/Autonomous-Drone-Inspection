#!/usr/bin/env bash
set -euo pipefail

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_TAG="${PX4_TAG:-v1.15.2}"
CUSTOM_GZ_WORLDS_DIR="${CUSTOM_GZ_WORLDS_DIR:-$HOME/gz_worlds}"
SRC_MODELS_DIR="${SRC_MODELS_DIR:-/home/uas/inspection/gazebo_simulation/models}"

# Fresh named volumes are mounted root-owned; make sure the user can write
# into the PX4 directory (clone + build) on first bootstrap.
if [ -d "$PX4_DIR" ] && [ ! -w "$PX4_DIR" ]; then
  echo "[bootstrap_px4] $PX4_DIR not writable — chowning to $(id -un)"
  sudo chown -R "$(id -un):$(id -gn)" "$PX4_DIR"
fi

if [ ! -f /home/uas/fastdds_udp.xml ]; then
  echo "[bootstrap_px4] Creating FastDDS UDP-only transport profile in /home/uas/fastdds_udp.xml"
  cat << 'EOF' > /home/uas/fastdds_udp.xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLStringName">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>udp_transport</transport_id>
            <type>UDPv4</type>
        </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="default_profile" is_default_profile="true">
        <rtps>
            <userTransports>
                <transport_id>udp_transport</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
EOF
fi

if [ ! -d "$PX4_DIR/.git" ]; then
  if [ -d "$PX4_DIR" ]; then
    echo "[bootstrap_px4] Clearing non-git files in $PX4_DIR"
    find "$PX4_DIR" -mindepth 1 -delete 2>/dev/null || true
  fi
  echo "[bootstrap_px4] Cloning PX4-Autopilot into $PX4_DIR"
  git clone https://github.com/PX4/PX4-Autopilot.git --depth=1 "$PX4_DIR"
fi

cd "$PX4_DIR"
PX4_GZ_MODELS_DIR="$PX4_DIR/Tools/simulation/gz/models"

echo "[bootstrap_px4] Fetching tags and checking out $PX4_TAG"
git fetch --tags --force

if git rev-parse "$PX4_TAG" >/dev/null 2>&1; then
  git checkout "$PX4_TAG"
else
  echo "[bootstrap_px4] Tag $PX4_TAG not found; staying on current branch"
fi

git submodule update --init --recursive

if [ -d "$SRC_MODELS_DIR" ]; then
  echo "[bootstrap_px4] Installing custom simulation models from $SRC_MODELS_DIR"
  mkdir -p "$PX4_GZ_MODELS_DIR"
  cp -rf "$SRC_MODELS_DIR"/. "$PX4_GZ_MODELS_DIR"/
fi

if [ -f "/home/uas/docker/fastdds_udp.xml" ] && \
   [ ! -f "/home/uas/fastdds_udp.xml" ]; then
  cp -f /home/uas/docker/fastdds_udp.xml /home/uas/fastdds_udp.xml
fi

if [ -d "$CUSTOM_GZ_WORLDS_DIR" ]; then
  echo "[bootstrap_px4] Installing custom Gazebo worlds from $CUSTOM_GZ_WORLDS_DIR"
  mkdir -p "$PX4_DIR/Tools/simulation/gz/worlds"
  cp -f "$CUSTOM_GZ_WORLDS_DIR"/*.sdf "$PX4_DIR/Tools/simulation/gz/worlds/" 2>/dev/null || true
fi

if [ -d "/home/uas/inspection/gazebo_simulation/models" ]; then
  echo "[bootstrap_px4] Installing custom Gazebo models"
  cp -rf /home/uas/inspection/gazebo_simulation/models/* "$PX4_DIR/Tools/simulation/gz/models/" 2>/dev/null || true
fi

if [ -f "$PX4_DIR/Tools/simulation/gz/models/OakD-Lite/model.sdf" ]; then
  echo "[bootstrap_px4] Optimizing OakD-Lite camera resolution to 640x480 for 30 FPS CPU rendering"
  sed -i 's/<width>1920<\/width>/<width>640<\/width>/g' "$PX4_DIR/Tools/simulation/gz/models/OakD-Lite/model.sdf"
  sed -i 's/<height>1080<\/height>/<height>480<\/height>/g' "$PX4_DIR/Tools/simulation/gz/models/OakD-Lite/model.sdf"
fi

mkdir -p "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix"
cat << 'EOF' > "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/px4-rc.params"
#!/bin/sh
# Keep the same contract as the documented PX4 SITL sensor defaults. The
# custom x500_depth model supplies IMU/barometer/GPS through Gazebo/PX4's
# simulation bridge; the simulated GPS and magnetometer must remain enabled.
param set-default SENS_EN_GPSSIM 1
param set-default SENS_EN_MAGSIM 1
param set-default SENS_EN_BAROSIM 0
param set COM_ARM_WO_GPS 1
param set COM_ARM_SWISBTN 1
param set COM_RC_IN_MODE 1
param set COM_RCL_EXCEPT 4
param set NAV_RCL_ACT 0
param set NAV_DLL_ACT 0
param set EKF2_GPS_CHECK 0
param set EKF2_MAG_CHECK 0
param set EKF2_REQ_NSATS 0
param set CBRK_SUPPLY_CHK 894281
param set CBRK_IO_SAFETY 22027
param set COM_ARM_MAG_STR 0
param set COM_DISARM_LAND 2.0
EOF

echo "[bootstrap_px4] Building PX4 SITL firmware"
make px4_sitl_default

echo "[bootstrap_px4] Build complete"
