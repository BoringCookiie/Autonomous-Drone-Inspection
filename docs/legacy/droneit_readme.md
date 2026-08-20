# UAS Infrastructure Monitoring - Phase 1

This folder provides the Dockerized baseline for the **Phase 1 prototype**:

- `sim_stack`: ROS2 Humble + Gazebo + PX4 SITL + MAVROS + RTAB-Map + OctoMap
- `ai_stack`: YOLO pipeline environment for dataset prep, training, and offline rosbag inference

## Folder Layout

- `docker/sim/Dockerfile` - simulation stack image
- `docker/ai/Dockerfile` - AI stack image
- `docker/sim/entrypoint.sh` - sim container entrypoint
- `docker/ai/entrypoint.sh` - AI container entrypoint
- `docker-compose.yml` - orchestration (profiles: `sim`, `ai`, `ai-gpu`; optional: `sim-headless`, `sim-wayland`, `sim-novnc`)
- `.env` - runtime variables (ROS domain, display, world)
- `scripts/bootstrap_px4.sh` - one-time PX4 clone/build helper
- `shared/` - shared workspace/data between containers
- `docs/gazebo-px4-docker-gpu.md` - Gazebo GPU in Docker, Docker Engine vs Desktop, X11, PX4 `gz` targets, noVNC limits
- `docs/setup.md` - **Obstacle-course SITL + MAVROS + `fly_pattern.py`** (Linux, Windows headless/noVNC, troubleshooting)
- `shared/gz_worlds/obstacle_course_final.sdf` - custom Gazebo world (mounted at `/home/uas/gz_worlds`, then copied into PX4 after clone)
- `scripts/run_obstacle_flight.sh`, `scripts/launch_obstacle_stack.sh`, `scripts/fly_pattern.py` - obstacle sim and autonomous demo

## Camera topics

The sim image builds `ros_gz_bridge` **from source (0.254.1, Gazebo Harmonic)** because the apt
`ros-humble-ros-gz-bridge` is Fortress-only and cannot decode PX4 SITL camera streams.
Launch with `PX4_GZ_MODEL_TARGET=gz_x500_mono_cam` (default in `launch_obstacle_stack.sh`) and the
bridge script publishes `/camera` + `/camera_info` into ROS 2 automatically.

## Quick Start

### 1) Build images

```bash
docker compose --profile sim --profile ai build
```

### 2) Start stacks

`.env` sets `COMPOSE_PROFILES=sim` so plain `docker compose up -d` starts the sim stack. Override profiles on the CLI when needed.

```bash
# default from .env (usually sim only)
docker compose up -d

# simulation only (explicit)
docker compose --profile sim up -d

# AI only (CPU — no NVIDIA runtime required)
docker compose --profile ai up -d

# AI with GPU (requires NVIDIA Container Toolkit + `nvidia` Docker runtime)
docker compose --profile ai-gpu up -d

# both CPU stacks
docker compose --profile sim --profile ai up -d

# Headless sim (no X11) — useful on servers or when host X11 sharing is unavailable
# Starts the headless sim service (`sim_headless`) which uses the same image
# but does not attempt to mount `/tmp/.X11-unix` or `/dev/dri`.
docker compose up -d sim_headless
```

### 3) One-time PX4 SITL bootstrap (inside sim container)

```bash
docker exec -it uas_sim bash
bash /home/uas/scripts/bootstrap_px4.sh
```

## Shared Data Paths

- Rosbags: `shared/rosbags`
- Datasets (SDNET2018 + MBDD2025): `shared/datasets`
- Trained weights: `shared/weights`
- ROS2 workspace: `shared/ros2_ws`

## Phase 1 Requirement Checklist

Use this as the hard gate for the 15 May prototype:

- [ ] P1: End-to-end simulation launch (PX4 + Gazebo + MAVROS + RTAB-Map)
- [ ] P2: Moroccan wall Gazebo world available and loadable
- [ ] P3: Coverage path mission executed and rosbag recorded
- [ ] P4: SDNET2018 + MBDD2025 downloaded and converted to YOLO format
- [ ] P5: One YOLOv11 model trained and used for offline rosbag inference
- [ ] P6: RViz2 overlay shows OctoMap + detected defects

## Operational Commands (Phase 1)

```bash
# PX4 SITL + Gazebo
make px4_sitl gz_x500

# Record all topics
ros2 bag record -a -o sim_flight_01

# YOLOv11 segmentation training
yolo train model=yolo11n-seg.pt data=dataset.yaml
```

## Gazebo + GPU on Linux

For hardware-accelerated Gazebo with **`sim_stack`**, use **native Docker Engine** (not Docker Desktop on Linux if you need reliable `/dev/dri`), allow X with **`xhost +local:`** on the host, and read **`docs/gazebo-px4-docker-gpu.md`** for DRI checks, **`sim` vs `sim-novnc`**, and PX4 **`gz_x500`** / submodule recovery.

## Notes

- Keep scope strict: only one AI model (YOLOv11) for Phase 1.
- Prefer offline inference on rosbag over real-time optimization.
- `ai_stack` runs on **CPU** by default so it starts without [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html). For GPU training, use **`docker compose --profile ai-gpu up -d`** (service `ai_stack_gpu`), which uses `runtime: nvidia` and fails fast if the toolkit is not configured.

### GUI troubleshooting

- If you see "mounts denied: /tmp/.X11-unix is not shared" you are likely using Docker Desktop (macOS/Windows) or your Docker daemon blocks host socket mounts.
	- On Docker Desktop: add `/tmp` or `/tmp/.X11-unix` under Preferences → Resources → File Sharing and restart Docker.
	- On Linux: if Docker is running rootless the daemon may block socket binds; either run the headless service above or run Docker as root / enable appropriate paths.
- **noVNC:** `docker compose --profile sim-novnc up -d --build sim_novnc`, then open **`http://127.0.0.1:6080/vnc.html`** (or `/vnc_lite.html`). The root URL alone often does not load the client. If the viewer is black, check `docker logs uas_sim_novnc` and inside the container `cat /tmp/xvnc.log /tmp/novnc.log /tmp/fluxbox.log`. The launch script is `scripts/novnc_sitl_launch.sh` (TigerVNC display `:1`, software GL by default).

### Wayland (GNOME 50) / XWayland instructions

If your host uses Wayland (GNOME 50) and XWayland, use the `sim_wayland` service which mounts the host Wayland socket and DRI. Start it with:

```bash
docker compose up -d sim_wayland
```

Notes and tips:
- Ensure your XDG runtime dir env is visible: `echo $XDG_RUNTIME_DIR` (usually `/run/user/1000`). The compose service uses `${XDG_RUNTIME_DIR}` if set, otherwise `/run/user/1000`.
- If apps still fail to display, check permissions on the Wayland socket and that your host user UID matches the container user (container user is UID 1000).
- If you prefer to run GUI apps with X11 compatibility, you can still try to share `/tmp/.X11-unix` (may require Docker Desktop file sharing or running Docker as root).



MVROS:
source /opt/ros/humble/setup.bash
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@127.0.0.1:14557

PX4:
   21  source /opt/ros/humble/setup.bash
   22  cd /home/uas/PX4-Autopilot
   23  export PX4_GZ_WORLD=default
   24  export PX4_GZ_MODEL=x500
   25  export GZ_SIM_RESOURCE_PATH="/home/uas/PX4-Autopilot/Tools/simulation/gz/models:$HOME/.gz/models:/usr/share/gz/gz-sim8/models"
   26  make px4_sitl gz_x500
   27  history
