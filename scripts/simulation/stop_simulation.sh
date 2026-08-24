#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CONTAINER="${UAS_SIM_CONTAINER:-uas_sim}"
SESSION="${UAS_OBSTACLE_TMUX_SESSION:-uas_obstacle}"

if tmux has-session -t "$SESSION" 2>/dev/null; then
  tmux kill-session -t "$SESSION"
  echo "stopped tmux session $SESSION"
fi

running() {
  # Docker's state endpoint can hiccup under load; retry before believing it.
  local attempt
  for attempt in 1 2 3; do
    if docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -qx true; then
      return 0
    fi
    sleep 2
  done
  return 1
}

if running; then
  docker exec "$CONTAINER" bash /home/uas/scripts/simulation/cleanup_runtime.sh
else
  echo "container $CONTAINER is not running"
fi

if [ "${1:-}" = "--down" ]; then
  docker compose --project-directory "$ROOT" \
    -f "$ROOT/docker/docker-compose.yml" --profile sim down
fi
