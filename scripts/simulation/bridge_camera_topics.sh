#!/usr/bin/env bash
set -euo pipefail

CONTAINER="${UAS_SIM_CONTAINER:-uas_sim}"

inner='
set -euo pipefail
echo "[bridge_camera_topics] Starting the native vehicle camera bridge..."
exec bash /home/uas/scripts/simulation/camera_bridge_native.sh
'

if docker inspect -f '{{.State.Running}}' "$CONTAINER" >/dev/null 2>&1; then
  if [[ -t 0 ]]; then
    exec docker exec -it "$CONTAINER" bash -lc "$inner"
  fi
  exec docker exec "$CONTAINER" bash -lc "$inner"
fi

exec bash -lc "$inner"
