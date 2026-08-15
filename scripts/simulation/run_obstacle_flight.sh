#!/bin/bash
set -euo pipefail

DOCKER_EXEC_FLAGS=()
if [[ -t 0 ]]; then
  DOCKER_EXEC_FLAGS=(-it)
fi

CONTAINER="${UAS_SIM_CONTAINER:-uas_sim}"
PX4_GZ_WORLD="${PX4_GZ_WORLD:-earthen_heritage_wall}"
PX4_GZ_MODEL_TARGET="${PX4_GZ_MODEL_TARGET:-gz_x500_depth}"
HEADLESS="${HEADLESS:-1}"

# Clean any stale px4 and gazebo processes and socket files inside container before launching
docker exec "$CONTAINER" bash -c "pkill -9 -f 'px4|gz|ruby|parameter_bridge|ninja|cmake' || true; rm -rf /tmp/gz* /tmp/ign* /tmp/px4*" || true

docker exec -i \
  -e PX4_GZ_WORLD="$PX4_GZ_WORLD" \
  -e PX4_GZ_MODEL_TARGET="$PX4_GZ_MODEL_TARGET" \
  -e HEADLESS="$HEADLESS" \
  "$CONTAINER" bash -c '
set -e
if [ ! -f /home/uas/PX4-Autopilot/Makefile ]; then
  echo "[run_obstacle_flight] PX4-Autopilot not found or incomplete — running bootstrap_px4.sh..."
  /home/uas/docker/bootstrap_px4.sh
fi
cd /home/uas/PX4-Autopilot
PX4_GZ_WORLD="${PX4_GZ_WORLD:-earthen_heritage_wall}"
PX4_GZ_MODEL_TARGET="${PX4_GZ_MODEL_TARGET:-gz_x500_depth}"
HEADLESS="${HEADLESS:-0}"

# Ensure worlds and models are accessible
mkdir -p /home/uas/PX4-Autopilot/Tools/simulation/gz/worlds
if compgen -G "/home/uas/gz_worlds/*.sdf" >/dev/null; then
  cp -f /home/uas/gz_worlds/*.sdf /home/uas/PX4-Autopilot/Tools/simulation/gz/worlds/
fi

export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export GALLIUM_DRIVER=llvmpipe
export PX4_GZ_WORLD
export HEADLESS
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/uas/fastdds_udp.xml
export GZ_SIM_RESOURCE_PATH="/home/uas/PX4-Autopilot/Tools/simulation/gz/models:/home/uas/inspection/gazebo_simulation/models:/home/uas/gz_worlds:$HOME/.gz/models:/usr/share/gz/gz-sim8/models"
export PYTHONPATH=${PYTHONPATH:-}:/home/uas/scripts

echo "[run_obstacle_flight] PX4_GZ_WORLD=$PX4_GZ_WORLD target=$PX4_GZ_MODEL_TARGET"
echo "[run_obstacle_flight] GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"
echo "[run_obstacle_flight] Starting Gazebo server in background..."
gz sim -r -s --headless-rendering "/home/uas/PX4-Autopilot/Tools/simulation/gz/worlds/${PX4_GZ_WORLD}.sdf" >/tmp/gz_server.log 2>&1 &

echo "[run_obstacle_flight] Waiting for Gazebo server to initialize..."
for i in $(seq 1 30); do
  if gz topic -l 2>/dev/null | grep -q "world/${PX4_GZ_WORLD}"; then
    echo "[run_obstacle_flight] Gazebo world ready (${i}s)"
    break
  fi
  sleep 1
done

if [ "$HEADLESS" = "0" ]; then
  echo "[run_obstacle_flight] Starting Gazebo 3D GUI window on display $DISPLAY..."
  gz sim -g >/tmp/gz_gui.log 2>&1 &
fi

echo "[run_obstacle_flight] Starting PX4 SITL..."
cd /home/uas/PX4-Autopilot/build/px4_sitl_default
rm -rf parameters* rootfs/fs/microsd/parameters* dataman
exec env PX4_SIM_MODEL="$PX4_GZ_MODEL_TARGET" bin/px4 -d
'
