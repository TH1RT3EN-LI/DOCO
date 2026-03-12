#!/usr/bin/env bash
set -euo pipefail

SCRIPT_NAME="$(basename "$0")"

DRY_RUN=0
FORCE_KILL=0
STOP_ROS2_DAEMON=1

usage() {
  cat <<EOF
Usage: ${SCRIPT_NAME} [--dry-run] [--force] [--keep-ros2-daemon]

Clean common lingering daemons/processes from the UAV SITL stack:
  - ros2 daemon
  - MicroXRCEAgent
  - PX4 SITL
  - gz sim / gz-server / gz-client

Options:
  --dry-run           Print what would be stopped without killing anything.
  --force             Skip graceful shutdown and send SIGKILL directly.
  --keep-ros2-daemon  Do not stop ros2 daemon.
  -h, --help          Show this help.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run)
      DRY_RUN=1
      ;;
    --force)
      FORCE_KILL=1
      ;;
    --keep-ros2-daemon)
      STOP_ROS2_DAEMON=0
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[${SCRIPT_NAME}] unknown option: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
  shift
done

log() {
  echo "[${SCRIPT_NAME}] $*"
}

process_state() {
  local pid="$1"
  ps -p "${pid}" -o stat= 2>/dev/null | awk '{print substr($1, 1, 1)}'
}

find_pids() {
  local pattern="$1"
  pgrep -f "${pattern}" || true
}

print_matches() {
  local label="$1"
  local pattern="$2"
  local pids

  pids="$(find_pids "${pattern}")"
  if [[ -z "${pids}" ]]; then
    return 0
  fi

  while IFS= read -r pid; do
    [[ -z "${pid}" ]] && continue
    state="$(process_state "${pid}")"
    log "matched ${label}: pid=${pid} state=${state:-?} cmd=$(ps -p "${pid}" -o args= 2>/dev/null || echo "<exited>")"
  done <<< "${pids}"
}

stop_pid() {
  local pid="$1"
  local label="$2"
  local state

  if [[ -z "${pid}" ]] || ! kill -0 "${pid}" 2>/dev/null; then
    return 0
  fi

  state="$(process_state "${pid}")"
  if [[ "${state}" == "Z" ]]; then
    log "skip zombie ${label}: pid=${pid}"
    return 0
  fi

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    log "would stop ${label}: pid=${pid}"
    return 0
  fi

  if [[ "${FORCE_KILL}" -eq 1 ]]; then
    log "SIGKILL ${label}: pid=${pid}"
    kill -9 "${pid}" 2>/dev/null || true
    return 0
  fi

  log "SIGTERM ${label}: pid=${pid}"
  kill "${pid}" 2>/dev/null || true

  for _ in {1..20}; do
    if ! kill -0 "${pid}" 2>/dev/null; then
      return 0
    fi
    sleep 0.2
  done

  log "SIGKILL ${label}: pid=${pid}"
  kill -9 "${pid}" 2>/dev/null || true
}

stop_pattern() {
  local label="$1"
  local pattern="$2"
  local pids

  pids="$(find_pids "${pattern}")"
  if [[ -z "${pids}" ]]; then
    return 0
  fi

  while IFS= read -r pid; do
    [[ -z "${pid}" ]] && continue
    stop_pid "${pid}" "${label}"
  done <<< "${pids}"
}

if [[ "${STOP_ROS2_DAEMON}" -eq 1 ]]; then
  if [[ "${DRY_RUN}" -eq 1 ]]; then
    print_matches "ros2 daemon" "[r]os2 daemon"
  else
    log "stopping ros2 daemon"
    ros2 daemon stop >/dev/null 2>&1 || true
  fi
fi

PROCESS_GROUPS=(
  "visual_landing_node|[v]isual_landing_node"
  "uav_visual_landing.sh|[u]av_visual_landing\\.sh"
  "uav_control_node|[u]av_control_node"
  "sitl_uav launch|[r]os2 launch .*sitl_uav\\.launch\\.py"
  "duojin sim launch|[r]os2 launch .*duojin01_bringup .*sim\\.launch\\.py"
  "MicroXRCEAgent|[M]icroXRCEAgent"
  "PX4 SITL|/px4($| )"
  "run_px4_gz_uav.sh|[r]un_px4_gz_uav\\.sh"
  "Gazebo Sim|[g]z sim"
  "Gazebo Server|[g]z-server"
  "Gazebo Client|[g]z-client"
)

for entry in "${PROCESS_GROUPS[@]}"; do
  label="${entry%%|*}"
  pattern="${entry#*|}"
  if [[ "${DRY_RUN}" -eq 1 ]]; then
    print_matches "${label}" "${pattern}"
  fi
  stop_pattern "${label}" "${pattern}"
done

if [[ "${DRY_RUN}" -eq 1 ]]; then
  log "dry-run complete"
else
  log "cleanup complete"
fi
