#!/bin/bash
set -euo pipefail

DOCKER_EXEC_FLAGS=()
if [[ -t 0 ]]; then
  DOCKER_EXEC_FLAGS=(-it)
fi

CONTAINER="${UAS_SIM_CONTAINER:-uas_sim}"
PX4_GZ_WORLD="${PX4_GZ_WORLD:-earthen_heritage_wall}"
PX4_GZ_MODEL_TARGET="${PX4_GZ_MODEL_TARGET:-gz_x500_mono_cam}"
PX4_GZ_MODEL_POSE="${PX4_GZ_MODEL_POSE:-}"
HEADLESS="${HEADLESS:-1}"

docker exec -i \
  -e PX4_GZ_WORLD="$PX4_GZ_WORLD" \
  -e PX4_GZ_MODEL_TARGET="$PX4_GZ_MODEL_TARGET" \
  -e PX4_GZ_MODEL_POSE="$PX4_GZ_MODEL_POSE" \
  -e HEADLESS="$HEADLESS" \
  "$CONTAINER" bash -c '
set -e
if [ ! -f /home/uas/PX4-Autopilot/Makefile ]; then
  echo "[run_obstacle_flight] PX4-Autopilot not found or incomplete — running bootstrap_px4.sh..."
  /home/uas/docker/bootstrap_px4.sh
fi
cd /home/uas/PX4-Autopilot
PX4_GZ_WORLD="${PX4_GZ_WORLD:-earthen_heritage_wall}"
PX4_GZ_MODEL_TARGET="${PX4_GZ_MODEL_TARGET:-gz_x500_mono_cam}"
HEADLESS="${HEADLESS:-0}"

# Ensure worlds and models are accessible
mkdir -p /home/uas/PX4-Autopilot/Tools/simulation/gz/worlds \
         /home/uas/PX4-Autopilot/Tools/simulation/gz/models
if compgen -G "/home/uas/gz_worlds/*.sdf" >/dev/null; then
  cp -f /home/uas/gz_worlds/*.sdf /home/uas/PX4-Autopilot/Tools/simulation/gz/worlds/
fi
if [ -d /home/uas/inspection/gazebo_simulation/models ]; then
  cp -rf /home/uas/inspection/gazebo_simulation/models/. /home/uas/PX4-Autopilot/Tools/simulation/gz/models/
fi

if [ "$PX4_GZ_MODEL_TARGET" = "gz_x500_depth" ] && {
   ! grep -q "sensor name=\"rgbd_camera\"" /home/uas/PX4-Autopilot/Tools/simulation/gz/models/x500_depth/model.sdf ||
   ! grep -q "child>rgbd_camera_link" /home/uas/PX4-Autopilot/Tools/simulation/gz/models/x500_depth/model.sdf
}; then
  echo "[run_obstacle_flight] x500_depth is missing its synchronized RGB-D camera assembly" >&2
  exit 1
fi

# Do not force llvmpipe. Gazebo Harmonic can use EGL headless rendering with
# the mounted DRI device; forcing a software OpenGL context can freeze camera
# render targets even while the physics/model pose continues to update.
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-0}"
unset MESA_GL_VERSION_OVERRIDE GALLIUM_DRIVER
export PX4_GZ_WORLD
export HEADLESS
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/uas/fastdds_udp.xml
export GZ_SIM_RESOURCE_PATH="/home/uas/PX4-Autopilot/Tools/simulation/gz/models:/home/uas/inspection/gazebo_simulation/models:/home/uas/gz_worlds:$HOME/.gz/models:/usr/share/gz/gz-sim8/models"
export PYTHONPATH=${PYTHONPATH:-}:/home/uas/scripts

echo "[run_obstacle_flight] PX4_GZ_WORLD=$PX4_GZ_WORLD target=$PX4_GZ_MODEL_TARGET"
echo "[run_obstacle_flight] GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"
echo "[run_obstacle_flight] Starting Gazebo server in background..."
GZ_SERVER_ARGS=(-r)
if [ "$HEADLESS" = "1" ]; then
  GZ_SERVER_ARGS+=(-s --headless-rendering)
fi
gz sim "${GZ_SERVER_ARGS[@]}" \
  "/home/uas/PX4-Autopilot/Tools/simulation/gz/worlds/${PX4_GZ_WORLD}.sdf" \
  >/tmp/gz_server.log 2>&1 &

echo "[run_obstacle_flight] Waiting for Gazebo server to initialize..."
for i in $(seq 1 30); do
  if gz topic -l 2>/dev/null | grep -q "world/${PX4_GZ_WORLD}"; then
    echo "[run_obstacle_flight] Gazebo world ready (${i}s)"
    break
  fi
  sleep 1
done

if ! gz topic -l 2>/dev/null | grep -q "world/${PX4_GZ_WORLD}"; then
  echo "[run_obstacle_flight] Gazebo world did not become ready" >&2
  cat /tmp/gz_server.log >&2 || true
  exit 1
fi

echo "[run_obstacle_flight] Starting PX4 SITL..."
cd /home/uas/PX4-Autopilot/build/px4_sitl_default
rm -rf parameters* rootfs/fs/microsd/parameters* rootfs/parameters*.bson dataman rootfs/dataman
env PX4_SIM_MODEL="$PX4_GZ_MODEL_TARGET" bin/px4 -d
echo "[run_obstacle_flight] PX4 SITL daemon started. Monitoring PX4 process..."
while pgrep -f "bin/px4" >/dev/null 2>&1 || pgrep -f "gz sim" >/dev/null 2>&1; do
  sleep 2
done
'
