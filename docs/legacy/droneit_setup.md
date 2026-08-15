# PX4 + Gazebo obstacle SITL and autonomous flight

This guide describes the GPU-capable Linux workflow (Fedora) and how teammates on Windows can run the same stack with headless or noVNC profiles.

## Repository layout (relevant pieces)

| Path | Role |
|------|------|
| `docker-compose.yml` | Sim/AI services; mounts `shared/gz_worlds/` at `/home/uas/gz_worlds` outside the PX4 checkout |
| `shared/gz_worlds/obstacle_course_final.sdf` | Validated world: default plugins + obstacles; center wall at **Y=4** so spawn at origin is clear |
| `scripts/fly_pattern.py` | MAVROS: optional param push, arm, OFFBOARD, pattern, `AUTO.LAND` — defaults to `/uas1` and can normalize `/uas1/mavros` |
| `scripts/run_obstacle_flight.sh` | Host script: `docker exec` into `uas_sim`, then `make px4_sitl gz_x500` with `PX4_GZ_WORLD=obstacle_course_final` + `GZ_SIM_RESOURCE_PATH` |
| `scripts/launch_obstacle_stack.sh` | Optional tmux (Linux): **window 0 = PX4+Gz**, 1 = MAVROS, 2 = `fly_pattern.py` (waits for services + FCU) |

## Docker Compose: world mount

The `sim_stack`, `sim_headless`, `sim_wayland`, and `sim_novnc` services include:

```yaml
- ./shared/gz_worlds:/home/uas/gz_worlds:ro
```

Do **not** bind-mount worlds directly inside `/home/uas/PX4-Autopilot`: that path is a Docker volume used for the PX4 clone/build. Mounting a file there makes the checkout non-empty/busy before `bootstrap_px4.sh` can clone PX4. `bootstrap_px4.sh` and `run_obstacle_flight.sh` copy `*.sdf` from `/home/uas/gz_worlds` into `Tools/simulation/gz/worlds/` after the PX4 checkout exists.

After changing the mount or compose file, recreate the container (e.g. `docker compose --profile sim up -d --force-recreate`).

## MAVROS namespace and “when it worked”

**What differed when things worked**

- Gazebo world included **full system plugins** + valid XML (see `shared/gz_worlds/obstacle_course_final.sdf`), so IMU/GPS/baro reached PX4.
- **`GZ_SIM_RESOURCE_PATH`** included `.../PX4-Autopilot/Tools/simulation/gz/models` so `gz_bridge` could spawn `x500`.
- **PX4 was running before** MAVROS and the Python script (no race on MAVLink port / plugins).
- Topics/services are expected under **`/uas1/...`** when `mavros_node` is launched with `-r __ns:=/uas1`. If `MAVROS_NS=/uas1/mavros` is set, `fly_pattern.py` strips the node name and uses `/uas1`.

**Environment variables (`fly_pattern.py`)**

| Variable | Purpose |
|----------|---------|
| `MAVROS_NS` | Force namespace (e.g. `/uas1`; `/uas1/mavros` is normalized to `/uas1`). |
| `FLY_PATTERN_SKIP_PARAM=1` | Do not use `param/set`; set pre-arm params in `pxh>` instead. |
| `FLY_PATTERN_PARAM_WAIT_SEC` | Seconds to wait for `param/set` (default `25`). |
| `FLY_PATTERN_UAS1_WAIT_SEC` | Seconds to wait for `/uas1/cmd/arming` before falling back to `/mavros` (default `90`). |

**`launch_obstacle_stack.sh`** starts **PX4+Gazebo first**, then MAVROS under `/uas1` (after 8s), then the flyer (waits until `/uas1/cmd/arming` exists, then sleeps 15s for heartbeat). Attach and use **Ctrl-b 0 / 1 / 2**.

## Linux (Fedora): one-time host prep

```bash
sudo systemctl enable --now docker
sudo usermod -aG docker "$USER"
newgrp docker   # or log out and back in
```

Allow the container to use the host display (and optionally GPU via existing compose/X11 setup):

```bash
sudo dnf install xorg-x11-server-utils
xhost +local:
```

Re-run `xhost +local:` after reboot if the GUI stops connecting.

## Linux: build and start sim

From the repository root (this `setup` directory):

```bash
docker compose build sim_stack
docker compose --profile sim up -d
```

## Linux: PX4 bootstrap (once, or after `docker compose down -v`)

PX4 lives in the Docker volume `px4_build`. Bootstrap clones PX4, installs the custom worlds from `/home/uas/gz_worlds`, and builds `gz_x500`:

```bash
docker exec -it uas_sim bash -lc "/home/uas/scripts/bootstrap_px4.sh"
```

Do **not** manually delete or clone `Tools/simulation/gz` inside the container. Custom worlds are mounted outside the PX4 checkout and copied in by `bootstrap_px4.sh` / `run_obstacle_flight.sh`.

## Linux: launch everything

**Option A — tmux (recommended on Linux)**

```bash
./scripts/launch_obstacle_stack.sh
```

This now launches the camera drone by default (`gz_x500_mono_cam`) and opens four windows:

```text
0 = PX4 + Gazebo
1 = MAVROS
2 = camera_bridge
3 = fly_pattern.py
```

Use **Ctrl-b** then **0**, **1**, **2**, or **3** to switch windows; **Ctrl-b d** to detach. Create session without attaching: `./scripts/launch_obstacle_stack.sh --no-attach`.

To use a different model:

```bash
PX4_GZ_MODEL_TARGET=gz_x500_depth ./scripts/launch_obstacle_stack.sh
PX4_GZ_MODEL_TARGET=gz_x500 ./scripts/launch_obstacle_stack.sh
```

To use the mini-maze world instead of the simpler obstacle course:

```bash
PX4_GZ_WORLD=obstacle_maze ./scripts/launch_obstacle_stack.sh
```

You can combine both:

```bash
PX4_GZ_WORLD=obstacle_maze PX4_GZ_MODEL_TARGET=gz_x500_mono_cam ./scripts/launch_obstacle_stack.sh
```

**Option B — three terminals**

1. **PX4 + Gazebo:** `PX4_GZ_WORLD=obstacle_maze PX4_GZ_MODEL_TARGET=gz_x500_mono_cam ./scripts/run_obstacle_flight.sh`
2. **MAVROS:**

   ```bash
   docker exec -it uas_sim bash
   source /opt/ros/humble/setup.bash
   ros2 run mavros mavros_node --ros-args -r __ns:=/uas1 -p fcu_url:=udp://:14540@127.0.0.1:14557
   ```

3. **Camera bridge:**

   ```bash
   ./scripts/bridge_camera_topics.sh
   ```

4. **Flight script:**

   ```bash
   docker exec -it uas_sim bash
   source /opt/ros/humble/setup.bash
   export MAVROS_NS=/uas1
   python3 /home/uas/scripts/fly_pattern.py
   ```

`fly_pattern.py` pushes the required SITL/offboard params automatically. Do not set `COM_ARM_EKF_*` to `0`; PX4 treats those as maximum innovation thresholds, and `0` makes arming stricter.

## Sensor data recording

`fly_pattern.py` automatically records a ROS 2 bag while the flight pattern runs. Bags are written inside the shared host folder:

```bash
shared/rosbags/fly_pattern_YYYYmmdd_HHMMSS/
```

By default it records **all ROS 2 topics** (`ros2 bag record -a`), including MAVROS IMU, GPS, local/global position, estimator status, altitude, setpoints, diagnostics, and any camera/lidar/depth topics that you bridge into ROS 2 separately with `ros_gz_bridge`.

Useful commands:

```bash
ros2 bag info /home/uas/rosbags/fly_pattern_YYYYmmdd_HHMMSS
ros2 bag play /home/uas/rosbags/fly_pattern_YYYYmmdd_HHMMSS
```

Readable analysis:

```bash
python3 /home/uas/scripts/analyze_rosbags.py --bag list
python3 /home/uas/scripts/analyze_rosbags.py --bag latest
python3 /home/uas/scripts/analyze_rosbags.py --bag latest --export-csv
python3 /home/uas/scripts/analyze_rosbags.py --bag latest --export-video
```

The analyzer prints flight mode changes, local position ranges, estimated travel distance, speed, IMU acceleration/gyro magnitudes, GPS/altitude, battery stats, most active topics, and any camera/image topics. CSV/video exports are written under:

```bash
/home/uas/rosbags/fly_pattern_YYYYmmdd_HHMMSS/analysis/
```

Environment variables:

| Variable | Purpose |
|----------|---------|
| `FLY_PATTERN_RECORD_BAG=0` | Disable sensor rosbag recording. |
| `FLY_PATTERN_BAG_DIR=/path` | Change output directory (default `/home/uas/rosbags`). |
| `FLY_PATTERN_BAG_TOPICS=/topic1,/topic2` | Record only specific topics instead of all ROS 2 topics. |

The base `gz_x500` model does not include a camera. To collect camera data, launch a camera-equipped PX4 model such as `gz_x500_mono_cam` and bridge the Gazebo camera topic into ROS 2; once it appears in `ros2 topic list`, the default bag recorder will capture it.

## Camera stream test

PX4 ships camera-equipped Gazebo models in this setup:

| Target | What it is for |
|--------|----------------|
| `gz_x500_mono_cam` | Lightweight RGB monocular camera test. |
| `gz_x500_depth` | OakD-Lite style RGB + depth/point cloud test. |

Launch PX4/Gazebo with a camera model:

```bash
PX4_GZ_MODEL_TARGET=gz_x500_mono_cam ./scripts/run_obstacle_flight.sh
```

In a second terminal, bridge Gazebo camera/depth topics into ROS 2:

```bash
./scripts/bridge_camera_topics.sh
```

Then start MAVROS and the flight script as usual. Because `fly_pattern.py` records all ROS 2 topics by default, bridged camera topics will be saved in the rosbag automatically. After the run:

```bash
docker exec -it uas_sim bash
source /opt/ros/humble/setup.bash
python3 /home/uas/scripts/analyze_rosbags.py --bag latest --export-video
```

If no video is produced, check whether Gazebo is publishing camera topics:

```bash
gz topic -l | grep -Ei "camera|image|depth|points"
ros2 topic list | grep -Ei "camera|image|depth|points"
```

## Windows (teammates)

The `sim` profile expects host X11 and `/dev/dri`; on Windows use **headless** or **noVNC**.

### Headless (`uas_sim_headless`)

```bash
docker compose --profile sim-headless up -d
```

Inside the container (replace `uas_sim` with `uas_sim_headless` in all `docker exec` targets):

```bash
docker exec -it uas_sim_headless bash
source /opt/ros/humble/setup.bash
cd /home/uas/PX4-Autopilot
export GZ_SIM_RESOURCE_PATH="/home/uas/PX4-Autopilot/Tools/simulation/gz/models:/usr/share/gz/gz-sim8/models:$HOME/.gz/models"
export PX4_GZ_WORLD=obstacle_course_final
export PX4_GZ_MODEL=x500
make px4_sitl gz_x500
```

Second terminal: same MAVROS command as Linux, with `uas_sim_headless`. Third: same `fly_pattern.py` flow. There is no local 3D window; physics and MAVROS still run.

### noVNC (browser GUI, software GL)

```bash
docker compose --profile sim-novnc up -d
```

Open `http://localhost:6080/vnc.html`. Use container name `uas_sim_novnc` in `docker exec` commands. Gazebo is typically slower here (`LIBGL_ALWAYS_SOFTWARE` may be set in that service).

### AI stack on Windows

Docker Desktop with WSL2 is the usual path. NVIDIA GPU + WSL2 + NVIDIA Container Toolkit can accelerate the **`ai-gpu`** profile; it does not replace Linux host X11 for the default `sim` GUI.

## Troubleshooting

| Symptom | What to check |
|--------|----------------|
| `PX4 server already running for instance 0` | A previous PX4 process is still alive. Stop it with `docker exec uas_sim bash -lc 'pkill -f "/home/uas/PX4-Autopilot/build/px4_sitl_default/bin/px4" || true; pkill -f "gz sim" || true'`, or `docker restart uas_sim`. |
| gz_bridge timeout, no model | `GZ_SIM_RESOURCE_PATH` includes `.../PX4-Autopilot/Tools/simulation/gz/models`; x500 model present under that tree |
| Missing sensors in world | World must include Gazebo system plugins; use `obstacle_course_final.sdf` from the repo |
| Drone crashes on arm (attitude / pitch) | Spawn inside geometry; wall is offset on Y in the provided world |
| Stuck on `param/set` or wrong MAVROS path | Start MAVROS with `-r __ns:=/uas1`; use `MAVROS_NS=/uas1`. The script uses `ParamSetV2` on `/uas1/param/set`. |
| Arming denied with estimator errors | Ensure `fly_pattern.py` sets `COM_ARM_EKF_YAW/POS/VEL/HGT` to `1.0`, not `0`. |
| `make px4_sitl gz_x500` submodule errors | Re-run `docker exec -it uas_sim bash -lc "/home/uas/scripts/bootstrap_px4.sh"` |
| No Gazebo window / cannot open display | Host: `xhost +local:`; `DISPLAY` forwarded (see `docker-compose.yml` and `docs/gazebo-px4-docker-gpu.md`) |
| Windows GUI for default `sim` | Use `sim-headless` or `sim-novnc`, not host X11 |
| Docker permission denied | User in `docker` group; new session after `usermod` |

## References

- [PX4 Gazebo simulation](https://docs.px4.io/main/en/sim_gazebo_gz/)
- [mavros (ROS 2)](https://github.com/mavlink/mavros)
- [Gazebo Fortress docs](https://gazebosim.org/docs/fortress)

See also `docs/gazebo-px4-docker-gpu.md` for GPU, X11, and Docker Engine vs Desktop on Linux.
