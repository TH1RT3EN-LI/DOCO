#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  uav_visual_landing.sh [--foreground] [-- <extra ros2 run args>]

Options:
  --foreground, -f   Run in foreground (default: background).
  --help, -h         Show this help.

Examples:
  ./ws/scripts/uav_visual_landing.sh
  ./ws/scripts/uav_visual_landing.sh --foreground
  ./ws/scripts/uav_visual_landing.sh -- --ros-args -p land_height_m:=0.08
USAGE
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SETUP_FILE="${WS_DIR}/install/setup.bash"
RUN_DIR="${WS_DIR}/.run"
LOG_DIR="${WS_DIR}/log/scripts"
PID_FILE="${RUN_DIR}/visual_landing_node.pid"
FOREGROUND=0
EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --foreground|-f)
      FOREGROUND=1
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    --)
      shift
      EXTRA_ARGS+=("$@")
      break
      ;;
    *)
      EXTRA_ARGS+=("$1")
      shift
      ;;
  esac
done

if [[ ! -f "${SETUP_FILE}" ]]; then
  echo "[uav_visual_landing] missing ${SETUP_FILE}" >&2
  echo "[uav_visual_landing] build first: cd ${WS_DIR} && colcon build" >&2
  exit 1
fi

# shellcheck disable=SC1090
set +u
source "${SETUP_FILE}"
set -u

mkdir -p "${RUN_DIR}" "${LOG_DIR}"

if [[ -f "${PID_FILE}" ]]; then
  old_pid="$(cat "${PID_FILE}")"
  if [[ -n "${old_pid}" ]] && kill -0 "${old_pid}" 2>/dev/null; then
    echo "[uav_visual_landing] already running (pid=${old_pid})"
    exit 0
  fi
  rm -f "${PID_FILE}"
fi

if [[ "${FOREGROUND}" -eq 1 ]]; then
  echo "[uav_visual_landing] starting in foreground"
  exec ros2 run uav_visual_landing visual_landing_node "${EXTRA_ARGS[@]}"
fi

ts="$(date +%Y%m%d_%H%M%S)"
log_file="${LOG_DIR}/visual_landing_${ts}.log"
echo "[uav_visual_landing] starting in background, log=${log_file}"
nohup ros2 run uav_visual_landing visual_landing_node "${EXTRA_ARGS[@]}" >"${log_file}" 2>&1 &
pid=$!
echo "${pid}" > "${PID_FILE}"
echo "[uav_visual_landing] started pid=${pid}"
