#!/usr/bin/env bash
set -euo pipefail

SCRIPT_NAME="$(basename "$0")"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SETUP_FILE="${WS_DIR}/install/setup.bash"
DEFAULT_WAIT_TIMEOUT_SEC="${UAV_VL_WAIT_TIMEOUT_SEC:-5}"
ACTION="start"
SERVICE_NAME=""
WAIT_TIMEOUT_SEC="${DEFAULT_WAIT_TIMEOUT_SEC}"

usage() {
  cat <<EOF_USAGE
Usage:
  ${SCRIPT_NAME} [start|stop] [service_name] [--timeout SEC]

Actions:
  start               Call visual landing start service (default).
  stop                Call visual landing stop service.

Arguments:
  service_name        Override the default ROS 2 service name.

Options:
  --timeout SEC       Wait up to SEC seconds for the service to appear.
  -h, --help          Show this help.

Default services:
  start -> /uav/visual_landing/start
  stop  -> /uav/visual_landing/stop

Notes:
  - In the current sim stack, these services are provided by
    visual_landing_node launched from ws/src/uav_bringup/launch/sitl_uav.launch.py,
    which is included by ws/src/duojin01_bringup/launch/sim.launch.py.
  - This script no longer launches a standalone node.

Examples:
  ./ws/scripts/uav_visual_landing.sh
  ./ws/scripts/uav_visual_landing.sh stop
  ./ws/scripts/uav_visual_landing.sh start --timeout 10
  ./ws/scripts/uav_visual_landing.sh stop /uav/visual_landing/stop
EOF_USAGE
}

log() {
  echo "[uav_visual_landing] $*"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    start|stop)
      ACTION="$1"
      shift
      ;;
    --timeout)
      if [[ $# -lt 2 ]]; then
        echo "[uav_visual_landing] missing value for --timeout" >&2
        usage >&2
        exit 1
      fi
      WAIT_TIMEOUT_SEC="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      if [[ -z "${SERVICE_NAME}" ]]; then
        SERVICE_NAME="$1"
        shift
      else
        echo "[uav_visual_landing] unexpected argument: $1" >&2
        usage >&2
        exit 1
      fi
      ;;
  esac
done

case "${ACTION}" in
  start)
    DEFAULT_SERVICE_NAME="/uav/visual_landing/start"
    ;;
  stop)
    DEFAULT_SERVICE_NAME="/uav/visual_landing/stop"
    ;;
  *)
    echo "[uav_visual_landing] unsupported action: ${ACTION}" >&2
    usage >&2
    exit 1
    ;;
esac

SERVICE_NAME="${SERVICE_NAME:-${DEFAULT_SERVICE_NAME}}"

if [[ ! -f "${SETUP_FILE}" ]]; then
  echo "[uav_visual_landing] missing ${SETUP_FILE}" >&2
  echo "[uav_visual_landing] build first: cd ${WS_DIR} && colcon build" >&2
  exit 1
fi

if ! [[ "${WAIT_TIMEOUT_SEC}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "[uav_visual_landing] invalid --timeout value: ${WAIT_TIMEOUT_SEC}" >&2
  exit 1
fi

set +u
# shellcheck source=/dev/null
source "${SETUP_FILE}"
set -u

wait_for_service() {
  local service_name="$1"
  local timeout_sec="$2"
  local timeout_ceil_sec
  local deadline_sec

  timeout_ceil_sec="$(awk -v timeout="${timeout_sec}" 'BEGIN { print (timeout == int(timeout)) ? int(timeout) : int(timeout) + 1 }')"
  deadline_sec=$((SECONDS + timeout_ceil_sec))

  while (( SECONDS <= deadline_sec )); do
    if ros2 service type "${service_name}" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.2
  done

  return 1
}

log "waiting for ${SERVICE_NAME} (timeout=${WAIT_TIMEOUT_SEC}s)"
if ! wait_for_service "${SERVICE_NAME}" "${WAIT_TIMEOUT_SEC}"; then
  echo "[uav_visual_landing] service not available: ${SERVICE_NAME}" >&2
  echo "[uav_visual_landing] start the sim stack first: ros2 launch duojin01_bringup sim.launch.py" >&2
  exit 1
fi

log "calling ${SERVICE_NAME}"
ros2 service call "${SERVICE_NAME}" std_srvs/srv/Trigger "{}"
