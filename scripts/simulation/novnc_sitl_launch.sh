#!/usr/bin/env bash
# Start TigerVNC (Xvnc), fluxbox, websockify (noVNC), then PX4 SITL + Gazebo GUI on :1.
# Intended for docker compose service sim_novnc only.
set -euo pipefail

export DISPLAY="${DISPLAY:-:1}"
export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
# Gazebo / Qt must use X11 inside TigerVNC (not Wayland / EGLFS).
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export GDK_BACKEND="${GDK_BACKEND:-x11}"
export SDL_VIDEODRIVER="${SDL_VIDEODRIVER:-x11}"
# Virtual framebuffer: llvmpipe avoids silent black window when DRI/GPU path fails in VNC.
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"

RES="${NOVNC_RESOLUTION:-1280x720}"
DEPTH="${NOVNC_DEPTH:-16}"
DISP_NUM="${DISPLAY#:}"
DISP_NUM="${DISP_NUM%%.*}"
XSOCK="/tmp/.X11-unix/X${DISP_NUM}"

mkdir -p /tmp/ccache-tmp
rm -f "/tmp/.X${DISP_NUM}-lock" "$XSOCK" || true

echo "[novnc_sitl] Starting Xvnc on ${DISPLAY} (${RES}, depth ${DEPTH})"
Xvnc "${DISPLAY}" -SecurityTypes None -localhost no -geometry "$RES" -depth "$DEPTH" >/tmp/xvnc.log 2>&1 &

for _ in $(seq 1 200); do
  [ -S "$XSOCK" ] && break
  sleep 0.05
done
if [ ! -S "$XSOCK" ]; then
  echo "[novnc_sitl] X socket missing: $XSOCK"
  tail -n 200 /tmp/xvnc.log || true
  exit 1
fi

VNC_PORT=$((5900 + DISP_NUM))
for _ in $(seq 1 150); do
  ss -ltn 2>/dev/null | grep -q ":${VNC_PORT}" && break
  sleep 0.05
done
if ! ss -ltn 2>/dev/null | grep -q ":${VNC_PORT}"; then
  echo "[novnc_sitl] VNC port ${VNC_PORT} not listening"
  tail -n 200 /tmp/xvnc.log || true
  exit 1
fi

echo "[novnc_sitl] Starting fluxbox + websockify (noVNC on 0.0.0.0:6080 -> localhost:${VNC_PORT})"
fluxbox >/tmp/fluxbox.log 2>&1 &
# Bind all interfaces so Docker published port 6080 reaches the process.
websockify --web=/usr/share/novnc "0.0.0.0:6080" "localhost:${VNC_PORT}" >/tmp/novnc.log 2>&1 &

sleep 0.3
echo "[novnc_sitl] Open in browser: http://127.0.0.1:6080/vnc.html (or /vnc_lite.html)"
echo "[novnc_sitl] Raw VNC (optional): localhost:${VNC_PORT}"

if [ ! -d "${HOME}/PX4-Autopilot/.git" ]; then
   /home/uas/docker/bootstrap_px4.sh
fi

cd "${HOME}/PX4-Autopilot"
exec make px4_sitl gz_x500
