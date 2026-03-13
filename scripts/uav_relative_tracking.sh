#!/usr/bin/env bash
set -euo pipefail

SCRIPT_NAME="$(basename "$0")"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SETUP_FILE="${WS_DIR}/install/setup.bash"
if [[ "${WS_DIR}" != "/ws" && -f "/ws/install/setup.bash" ]]; then
  SETUP_FILE="/ws/install/setup.bash"
fi
DEFAULT_WAIT_TIMEOUT_SEC="${UAV_RELATIVE_TRACKING_WAIT_TIMEOUT_SEC:-5}"
DEFAULT_READY_TIMEOUT_SEC="${UAV_RELATIVE_TRACKING_READY_TIMEOUT_SEC:-30}"
READY_STATUS_TOPIC="${UAV_RELATIVE_TRACKING_READY_TOPIC:-/relative_position/relocalize_requested}"
DIAGNOSTICS_TOPIC="${UAV_RELATIVE_TRACKING_DIAGNOSTICS_TOPIC:-/relative_position/diagnostics}"
WAIT_FOR_READY="${UAV_RELATIVE_TRACKING_WAIT_FOR_READY:-true}"

ACTION="start"
TARGET_HEIGHT_M="${UAV_RELATIVE_TRACKING_HEIGHT_M:-1.5}"
SERVICE_NAME=""
WAIT_TIMEOUT_SEC="${DEFAULT_WAIT_TIMEOUT_SEC}"
READY_TIMEOUT_SEC="${DEFAULT_READY_TIMEOUT_SEC}"
LAST_RELOCALIZATION_REASON=""

usage() {
  cat <<EOF_USAGE
Usage:
  ${SCRIPT_NAME} [start|position_mode|go_above_ugv|stop] [args] [service_name] [--timeout SEC] [--ready-timeout SEC]

Actions:
  start                Call relative tracking start service.
                       Optional arg: target_height_m
  position_mode        Switch UAV to manual position mode.
  go_above_ugv         Resume tracking and return to 1.0 m above UGV.
  stop                 Stop relative tracking and request hold.

Arguments:
  target_height_m      Only for start. Default: ${TARGET_HEIGHT_M}
  service_name         Override the default ROS 2 service name.

Options:
  --timeout SEC        Wait up to SEC seconds for the service to appear.
  --ready-timeout SEC  Only for start. Wait up to SEC seconds for fusion readiness.
  --no-ready-wait      Only for start. Skip waiting for fusion readiness.
  -h, --help           Show this help.

Default services:
  start         -> /uav/relative_tracking/command/start
  position_mode -> /uav/relative_tracking/command/position_mode
  go_above_ugv  -> /uav/relative_tracking/command/go_above_ugv
  stop          -> /uav/relative_tracking/command/stop

Examples:
  ./ws/scripts/uav_relative_tracking.sh
  ./ws/scripts/uav_relative_tracking.sh start 1.2
  ./ws/scripts/uav_relative_tracking.sh position_mode
  ./ws/scripts/uav_relative_tracking.sh go_above_ugv
  ./ws/scripts/uav_relative_tracking.sh stop
EOF_USAGE
}

log() {
  echo "[uav_relative_tracking] $*"
}

is_number() {
  [[ "$1" =~ ^[-+]?[0-9]+([.][0-9]+)?$ ]]
}

is_true() {
  case "$1" in
    1|true|TRUE|yes|YES|on|ON)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

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

fetch_ready_state() {
  timeout 1s ros2 topic echo \
    --once \
    --qos-profile system_default \
    --field data \
    "${READY_STATUS_TOPIC}" \
    std_msgs/msg/Bool 2>/dev/null |
    tr '[:upper:]' '[:lower:]' |
    grep -Eom1 'true|false' || true
}

fetch_relocalization_reason() {
  timeout 1s ros2 topic echo \
    --once \
    --qos-profile system_default \
    "${DIAGNOSTICS_TOPIC}" \
    diagnostic_msgs/msg/DiagnosticArray 2>/dev/null | awk '
      /^[[:space:]]+- key: relocalization_reason$/ {
        expect_reason_value = 1
        next
      }
      expect_reason_value && /^[[:space:]]+value: / {
        sub(/^[[:space:]]+value: /, "")
        gsub(/^'\''|'\''$/, "")
        print
        found_reason = 1
        exit
      }
      /^[[:space:]]+message: / && !found_message {
        fallback_message = $0
        sub(/^[[:space:]]+message: /, "", fallback_message)
        gsub(/^'\''|'\''$/, "", fallback_message)
        found_message = 1
      }
      END {
        if (!found_reason && found_message) {
          print fallback_message
        }
      }' || true
}

wait_for_ready() {
  local timeout_sec="$1"
  local timeout_ceil_sec
  local deadline_sec
  local last_reason=""

  timeout_ceil_sec="$(awk -v timeout="${timeout_sec}" 'BEGIN { print (timeout == int(timeout)) ? int(timeout) : int(timeout) + 1 }')"
  deadline_sec=$((SECONDS + timeout_ceil_sec))

  while (( SECONDS <= deadline_sec )); do
    local ready_state
    ready_state="$(fetch_ready_state)"
    if [[ "${ready_state}" == "false" ]]; then
      LAST_RELOCALIZATION_REASON=""
      return 0
    fi

    local reason
    reason="$(fetch_relocalization_reason)"
    if [[ -n "${reason}" && "${reason}" != "${last_reason}" ]]; then
      log "relative position fusion not ready yet (${reason})"
      last_reason="${reason}"
    fi

    sleep 0.5
  done

  LAST_RELOCALIZATION_REASON="${last_reason}"
  return 1
}

service_response_failed() {
  grep -Eq 'success(=|:[[:space:]]*)(False|false)' <<<"$1"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    start|position_mode|go_above_ugv|stop)
      ACTION="$1"
      shift
      ;;
    --timeout)
      if [[ $# -lt 2 ]]; then
        echo "[uav_relative_tracking] missing value for --timeout" >&2
        usage >&2
        exit 1
      fi
      WAIT_TIMEOUT_SEC="$2"
      shift 2
      ;;
    --ready-timeout)
      if [[ $# -lt 2 ]]; then
        echo "[uav_relative_tracking] missing value for --ready-timeout" >&2
        usage >&2
        exit 1
      fi
      READY_TIMEOUT_SEC="$2"
      shift 2
      ;;
    --no-ready-wait)
      WAIT_FOR_READY="false"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      if [[ "${ACTION}" == "start" && -z "${SERVICE_NAME}" ]] && is_number "$1"; then
        TARGET_HEIGHT_M="$1"
        shift
      elif [[ -z "${SERVICE_NAME}" ]]; then
        SERVICE_NAME="$1"
        shift
      else
        echo "[uav_relative_tracking] unexpected argument: $1" >&2
        usage >&2
        exit 1
      fi
      ;;
  esac
done

case "${ACTION}" in
  start)
    DEFAULT_SERVICE_NAME="/uav/relative_tracking/command/start"
    SERVICE_TYPE="relative_position_fusion/srv/StartRelativeTracking"
    REQUEST="{target_height_m: ${TARGET_HEIGHT_M}}"
    ;;
  position_mode)
    DEFAULT_SERVICE_NAME="/uav/relative_tracking/command/position_mode"
    SERVICE_TYPE="std_srvs/srv/Trigger"
    REQUEST="{}"
    ;;
  go_above_ugv)
    DEFAULT_SERVICE_NAME="/uav/relative_tracking/command/go_above_ugv"
    SERVICE_TYPE="std_srvs/srv/Trigger"
    REQUEST="{}"
    ;;
  stop)
    DEFAULT_SERVICE_NAME="/uav/relative_tracking/command/stop"
    SERVICE_TYPE="std_srvs/srv/Trigger"
    REQUEST="{}"
    ;;
  *)
    echo "[uav_relative_tracking] unsupported action: ${ACTION}" >&2
    usage >&2
    exit 1
    ;;
esac

SERVICE_NAME="${SERVICE_NAME:-${DEFAULT_SERVICE_NAME}}"

if [[ ! -f "${SETUP_FILE}" ]]; then
  echo "[uav_relative_tracking] missing ${SETUP_FILE}" >&2
  echo "[uav_relative_tracking] build first: cd ${WS_DIR} && colcon build" >&2
  exit 1
fi

if ! [[ "${WAIT_TIMEOUT_SEC}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "[uav_relative_tracking] invalid --timeout value: ${WAIT_TIMEOUT_SEC}" >&2
  exit 1
fi

if [[ "${ACTION}" == "start" ]] && is_true "${WAIT_FOR_READY}" &&
  ! [[ "${READY_TIMEOUT_SEC}" =~ ^[0-9]+([.][0-9]+)?$ ]]
then
  echo "[uav_relative_tracking] invalid --ready-timeout value: ${READY_TIMEOUT_SEC}" >&2
  exit 1
fi

if [[ "${ACTION}" == "start" ]] && ! is_number "${TARGET_HEIGHT_M}"; then
  echo "[uav_relative_tracking] invalid target_height_m: ${TARGET_HEIGHT_M}" >&2
  exit 1
fi

set +u
# shellcheck source=/dev/null
source "${SETUP_FILE}"
set -u

log "waiting for ${SERVICE_NAME} (timeout=${WAIT_TIMEOUT_SEC}s)"
if ! wait_for_service "${SERVICE_NAME}" "${WAIT_TIMEOUT_SEC}"; then
  echo "[uav_relative_tracking] service not available: ${SERVICE_NAME}" >&2
  echo "[uav_relative_tracking] start the sim stack first:" >&2
  echo "[uav_relative_tracking]   ros2 launch duojin01_bringup sim.launch.py" >&2
  echo "[uav_relative_tracking]   or" >&2
  echo "[uav_relative_tracking]   ros2 launch duojin01_bringup sim_navigation.launch.py" >&2
  exit 1
fi

if [[ "${ACTION}" == "start" ]] && is_true "${WAIT_FOR_READY}"; then
  log "waiting for relative position fusion readiness on ${READY_STATUS_TOPIC} (timeout=${READY_TIMEOUT_SEC}s)"
  if ! wait_for_ready "${READY_TIMEOUT_SEC}"; then
    echo "[uav_relative_tracking] relative position fusion is not ready" >&2
    if [[ -n "${LAST_RELOCALIZATION_REASON}" ]]; then
      echo "[uav_relative_tracking] relocalization_reason: ${LAST_RELOCALIZATION_REASON}" >&2
    else
      echo "[uav_relative_tracking] no readiness update received from ${READY_STATUS_TOPIC}" >&2
    fi
    exit 1
  fi
fi

log "calling ${SERVICE_NAME}"
if ! SERVICE_OUTPUT="$(ros2 service call "${SERVICE_NAME}" "${SERVICE_TYPE}" "${REQUEST}" 2>&1)"; then
  printf '%s\n' "${SERVICE_OUTPUT}"
  exit 1
fi
printf '%s\n' "${SERVICE_OUTPUT}"

if service_response_failed "${SERVICE_OUTPUT}"; then
  if [[ "${ACTION}" == "start" ]]; then
    LAST_RELOCALIZATION_REASON="$(fetch_relocalization_reason)"
    if [[ -n "${LAST_RELOCALIZATION_REASON}" ]]; then
      echo "[uav_relative_tracking] relocalization_reason: ${LAST_RELOCALIZATION_REASON}" >&2
    fi
  fi
  exit 1
fi
