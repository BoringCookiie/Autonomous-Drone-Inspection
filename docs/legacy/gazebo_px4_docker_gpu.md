# Gazebo + PX4 in Docker: GPU acceleration and common failures

This note captures what usually goes wrong when Gazebo is slow or invisible in containers, and how this repo is set up to avoid it. Authoritative compose fields live in **`docker-compose.yml`** (service **`sim_stack`**); treat the snippets below as explanations, not a second source of truth.

---

## 1. Why Gazebo feels laggy: software vs hardware OpenGL

Without access to the host GPU, Mesa typically falls back to **llvmpipe** (CPU software rendering). Gazebo then becomes heavy and unresponsive.

**Goal:** expose the host **DRI** devices (`/dev/dri`, e.g. `card0` / `card1`, `renderD128`, …) to the container and keep **`LIBGL_ALWAYS_SOFTWARE=0`** so Mesa can use the real driver when possible.

---

## 2. Docker Desktop on Linux vs native Engine

**Docker Desktop** runs the engine inside a VM. The VM’s `/dev` tree is not your host’s; host GPUs are often **invisible** or awkward to pass through, no matter how you configure compose.

For reliable GPU access on **Fedora (and most Linux)**, use **Docker Engine from Docker’s repo** (or distro packages that run on the host), not Desktop:

```bash
# Stop Desktop if it was installed
sudo systemctl stop docker-desktop 2>/dev/null || true

# Example: Docker CE on Fedora (see Docker’s current install docs for updates)
sudo dnf install -y dnf-plugins-core
sudo dnf config-manager --add-repo https://download.docker.com/linux/fedora/docker-ce.repo
sudo dnf install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo systemctl enable --now docker
sudo usermod -aG docker "$USER"
```

Log out and back in (or `newgrp docker`) so the **`docker`** group applies.

**Credential helper noise:** if pulls fail with `docker-credential-desktop` missing, fix or rename **`~/.docker/config.json`** (remove or change `credsStore` / `credHelpers` left over from Desktop).

---

## 3. What `sim_stack` does for the GPU

The **`sim`** profile service **`sim_stack`** is intended for **host X11** (or XWayland) plus hardware GL:

- **`privileged: true`** — matches what many PX4 + SITL flows expect for device and process behavior (see compose file).
- **`volumes: /dev/dri:/dev/dri`** — whole DRI tree inside the container (works across `card0` vs `card1` naming).
- **`LIBGL_ALWAYS_SOFTWARE=0`** in environment — request hardware OpenGL when DRI is available.

**Check inside the container:**

```bash
docker exec -it uas_sim bash -lc 'ls -la /dev/dri && (command -v glxinfo >/dev/null && glxinfo -B | grep -E "Device|renderer" || echo "install mesa-utils for glxinfo")'
```

You want a real GPU name (e.g. Intel / AMD), not **llvmpipe**, when software rendering is off. The sim image already includes Mesa userspace packages (`libgl1-mesa-dri`, etc.); if `glxinfo` still shows llvmpipe, fix DRI visibility first, then driver packages.

---

## 4. `sim-novnc` vs hardware acceleration

The **`sim_novnc`** profile runs **TigerVNC (Xvnc)** *inside* the container. That is a **virtual framebuffer X server**. OpenGL in that path is effectively **off the host’s composited display**; for smooth 3D people often use **VirtualGL** or similar, which this repo does **not** configure.

By design, **`sim_novnc`** sets **`LIBGL_ALWAYS_SOFTWARE=1`** for a more predictable browser viewer. Use it when you need a **web UI** without host X11, not when you need maximum GPU performance.

For **local GPU-accelerated Gazebo**, use **`--profile sim`** and your normal desktop session (XWayland on Wayland hosts), not **`sim-novnc`**.

---

## 5. Host X11 permission

Allow the container user to connect to your display (run on the **host** before starting GUI sim):

```bash
xhost +local:
```

Tighter variants exist (`xhost +SI:localuser:…`); `+local:` is a common compromise on a single-user workstation.

---

## 6. PX4 + Gazebo Harmonic: submodules and targets

PX4 **1.15** SITL targets **Gazebo Harmonic** (`gz`), not Gazebo Classic. Use **`make px4_sitl gz_x500`** (or the model your team chose), not legacy **`gazebo`** targets.

If **`Tools/simulation/gz`** is empty, wrong, or CMake complains about missing files / submodule drift:

```bash
docker exec -it uas_sim bash
cd ~/PX4-Autopilot
rm -rf Tools/simulation/gz
git clone https://github.com/PX4/PX4-gazebo-models.git Tools/simulation/gz
git submodule update --init --recursive   # when prompted by make, follow PX4’s guidance (e.g. sync submodules)
source /opt/ros/humble/setup.bash
make px4_sitl gz_x500
```

Prefer a clean tree matching **`scripts/bootstrap_px4.sh`** (pinned tag) before heavy surgery; the block above is for **broken or partial checkouts** after failed clones.

---

## 7. Steady-state workflow (GPU sim on host display)

From the repo root:

```bash
docker compose --profile sim up -d
docker exec -it uas_sim bash -lc '
  source /opt/ros/humble/setup.bash
  cd ~/PX4-Autopilot
  make px4_sitl gz_x500
'
```

Stop:

```bash
docker compose --profile sim down
```

After a successful build, you may run binaries manually without rebuilding every time; see PX4 docs for your exact launch sequence and env vars (`PX4_GZ_MODEL`, `PX4_GZ_WORLD`, etc.).

---

## 8. Pitfalls (short list)

| Avoid | Why |
|--------|-----|
| Relying on **Docker Desktop on Linux** for DRI | VM boundary hides host GPU. |
| Expecting **GPU perf from `sim-novnc`** | Xvnc path is for convenience; defaults lean software GL. |
| **`make px4_sitl gazebo`** (Classic) | Wrong simulator line for Harmonic / `gz` workflows. |
| **`LIBGL_ALWAYS_SOFTWARE=1`** on **`sim_stack`** when you want the GPU | Forces llvmpipe if left on by mistake. |
| Skipping **`xhost`** on first GUI run | X connection refused from the container. |

**ROS-GZ packages:** this image already installs **`gz-harmonic`** and **`ros-humble-ros-gzharmonic`** for the ROS 2 + Gazebo stack. Do not add conflicting `ros-gz` variants ad hoc without checking versions against PX4 and this Dockerfile.

---

## 9. Where to look next

- **`docker-compose.yml`** — `sim_stack`, `sim_wayland`, `sim_novnc` (DRI / display / novnc differ per profile).
- **`docker/sim/Dockerfile`** — Mesa, Gazebo Harmonic, VNC stack.
- **`scripts/bootstrap_px4.sh`** — first-time PX4 clone + build into the `px4_build` volume.

When in doubt, reproduce with **`docker compose --profile sim up -d`**, **`xhost +local:`**, then **`glxinfo`** and **`ls /dev/dri`** inside **`uas_sim`** before debugging application code.
