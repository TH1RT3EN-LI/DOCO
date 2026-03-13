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
DEFAULT_TAKEOFF_SERVICE_NAME="${UAV_RELATIVE_TRACKING_TAKEOFF_SERVICE:-/uav/control/command/takeoff}"
DEFAULT_TAKEOFF_READY_TIMEOUT_SEC="${UAV_RELATIVE_TRACKING_TAKEOFF_READY_TIMEOUT_SEC:-15}"
DEFAULT_TAKEOFF_READY_DELTA_M="${UAV_RELATIVE_TRACKING_TAKEOFF_READY_DELTA_M:-0.6}"
POSITION_MODE_CONTROL_SERVICE="${UAV_RELATIVE_TRACKING_POSITION_MODE_CONTROL_SERVICE:-/uav/control/command/position_mode}"
READY_STATUS_TOPIC="${UAV_RELATIVE_TRACKING_READY_TOPIC:-/relative_position/relocalize_requested}"
DIAGNOSTICS_TOPIC="${UAV_RELATIVE_TRACKING_DIAGNOSTICS_TOPIC:-/relative_position/diagnostics}"
UAV_STATE_TOPIC="${UAV_RELATIVE_TRACKING_UAV_STATE_TOPIC:-/uav/state/odometry}"
WAIT_FOR_READY="${UAV_RELATIVE_TRACKING_WAIT_FOR_READY:-true}"
USE_TAKEOFF_ON_START="${UAV_RELATIVE_TRACKING_USE_TAKEOFF_ON_START:-true}"

ACTION="start"
TARGET_HEIGHT_M="${UAV_RELATIVE_TRACKING_HEIGHT_M:-1.5}"
SERVICE_NAME=""
TAKEOFF_SERVICE_NAME="${DEFAULT_TAKEOFF_SERVICE_NAME}"
WAIT_TIMEOUT_SEC="${DEFAULT_WAIT_TIMEOUT_SEC}"
READY_TIMEOUT_SEC="${DEFAULT_READY_TIMEOUT_SEC}"
TAKEOFF_READY_TIMEOUT_SEC="${DEFAULT_TAKEOFF_READY_TIMEOUT_SEC}"
TAKEOFF_READY_DELTA_M="${DEFAULT_TAKEOFF_READY_DELTA_M}"
LAST_RELOCALIZATION_REASON=""
LAST_TAKEOFF_ALTITUDE_M=""
LAST_TAKEOFF_ALTITUDE_DELTA_M=""
LAST_TAKEOFF_WAIT_SAMPLE_COUNT=0

usage() {
  cat <<EOF_USAGE
Usage:
  ${SCRIPT_NAME} [start|position_mode|go_above_ugv|stop] [args] [service_name] [--timeout SEC] [--ready-timeout SEC]

Actions:
  start                Call takeoff service, wait until airborne, then start relative tracking.
                       Optional arg: target_height_m
  position_mode        Stop relative tracking and switch PX4/QGC to position mode.
  go_above_ugv         Resume tracking and return to 1.0 m above UGV.
  stop                 Stop relative tracking and request hold.

Arguments:
  target_height_m      Only for start. Default: ${TARGET_HEIGHT_M}
  service_name         Override the default ROS 2 service name.

Options:
  --timeout SEC        Wait up to SEC seconds for the service to appear.
  --ready-timeout SEC  Only for start. Wait up to SEC seconds for fusion readiness.
  --takeoff-service NAME
                       Only for start. Override the takeoff service.
  --takeoff-ready-timeout SEC
                       Only for start. Wait up to SEC seconds for the UAV to climb after takeoff.
  --takeoff-ready-delta M
                       Only for start. Minimum climb delta before relative tracking start.
  --skip-takeoff       Only for start. Skip calling the takeoff service first.
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
  ./ws/scripts/uav_relative_tracking.sh start 1.2 --takeoff-ready-delta 0.5
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

is_ignorable_token() {
  [[ "$1" =~ ^[[:space:][:punct:]，。！？；：、（）【】《》￥…“”‘’]+$ ]]
}

looks_like_ros_name() {
  [[ "$1" =~ ^[~/A-Za-z][A-Za-z0-9_/]*$ ]]
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

fetch_relocalize_requested_state() {
  timeout 1s ros2 topic echo \
    --once \
    --qos-profile system_default \
    --field data \
    "${READY_STATUS_TOPIC}" \
    std_msgs/msg/Bool 2>/dev/null |
    tr '[:upper:]' '[:lower:]' |
    grep -Eom1 'true|false' || true
}

fetch_diagnostics_value() {
  local key_name="$1"

  timeout 1s ros2 topic echo \
    --once \
    --qos-profile system_default \
    "${DIAGNOSTICS_TOPIC}" \
    diagnostic_msgs/msg/DiagnosticArray 2>/dev/null | awk -v target_key="${key_name}" '
      /^[[:space:]]+- key: / {
        current_key = $0
        sub(/^[[:space:]]+- key: /, "", current_key)
        gsub(/^'\''|'\''$/, "", current_key)
        expect_value = (current_key == target_key)
        next
      }
      expect_value && /^[[:space:]]+value: / {
        value = $0
        sub(/^[[:space:]]+value: /, "", value)
        gsub(/^'\''|'\''$/, "", value)
        print value
        exit
      }' || true
}

fetch_fusion_ready_state() {
  local initialized_state
  local relocalize_requested_state

  initialized_state="$(fetch_diagnostics_value initialized | tr '[:upper:]' '[:lower:]')"
  relocalize_requested_state="$(fetch_diagnostics_value relocalize_requested | tr '[:upper:]' '[:lower:]')"

  if [[ "${initialized_state}" == "true" && "${relocalize_requested_state}" == "false" ]]; then
    printf 'ready\n'
    return 0
  fi

  if [[ -n "${initialized_state}" || -n "${relocalize_requested_state}" ]]; then
    printf 'not_ready\n'
    return 0
  fi

  relocalize_requested_state="$(fetch_relocalize_requested_state)"
  if [[ "${relocalize_requested_state}" == "false" ]]; then
    printf 'not_ready\n'
    return 0
  fi

  return 1
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

extract_first_number() {
  awk '
    {
      for (i = 1; i <= NF; ++i) {
        if ($i ~ /^[-+]?[0-9]+([.][0-9]+)?([eE][-+]?[0-9]+)?$/) {
          print $i
          exit
        }
      }
    }'
}

fetch_uav_altitude() {
  local altitude

  for qos_profile in sensor_data system_default ""; do
    if [[ -n "${qos_profile}" ]]; then
      altitude="$(timeout 1s ros2 topic echo \
        --once \
        --qos-profile "${qos_profile}" \
        --field pose.pose.position.z \
        "${UAV_STATE_TOPIC}" \
        nav_msgs/msg/Odometry 2>/dev/null | extract_first_number || true)"
    else
      altitude="$(timeout 1s ros2 topic echo \
        --once \
        --field pose.pose.position.z \
        "${UAV_STATE_TOPIC}" \
        nav_msgs/msg/Odometry 2>/dev/null | extract_first_number || true)"
    fi
    if [[ -n "${altitude}" ]]; then
      printf '%s\n' "${altitude}"
      return 0
    fi
  done

  for qos_profile in sensor_data system_default ""; do
    if [[ -n "${qos_profile}" ]]; then
      altitude="$(timeout 1s ros2 topic echo \
        --once \
        --qos-profile "${qos_profile}" \
        "${UAV_STATE_TOPIC}" \
        nav_msgs/msg/Odometry 2>/dev/null | awk '
          /^pose:$/ {
            in_pose = 1
            next
          }
          in_pose && /^[^[:space:]]/ {
            exit
          }
          in_pose && /^[[:space:]]+position:$/ {
            in_position = 1
            next
          }
          in_pose && in_position && /^[[:space:]]+z:[[:space:]]*/ {
            sub(/^[[:space:]]+z:[[:space:]]*/, "")
            print
            exit
          }' || true)"
    else
      altitude="$(timeout 1s ros2 topic echo \
        --once \
        "${UAV_STATE_TOPIC}" \
        nav_msgs/msg/Odometry 2>/dev/null | awk '
      /^pose:$/ {
        in_pose = 1
        next
      }
      in_pose && /^[^[:space:]]/ {
        exit
      }
      in_pose && /^[[:space:]]+position:$/ {
        in_position = 1
        next
      }
      in_pose && in_position && /^[[:space:]]+z:[[:space:]]*/ {
        sub(/^[[:space:]]+z:[[:space:]]*/, "")
        print
        exit
      }' || true)"
    fi
    if [[ -n "${altitude}" ]]; then
      printf '%s\n' "${altitude}"
      return 0
    fi
  done

  return 1
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
    ready_state="$(fetch_fusion_ready_state || true)"
    if [[ "${ready_state}" == "ready" ]]; then
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

wait_for_takeoff_climb() {
  local initial_altitude_m="$1"
  local required_delta_m="$2"
  local timeout_sec="$3"
  local timeout_ceil_sec
  local deadline_sec
  LAST_TAKEOFF_WAIT_SAMPLE_COUNT=0

  timeout_ceil_sec="$(awk -v timeout="${timeout_sec}" 'BEGIN { print (timeout == int(timeout)) ? int(timeout) : int(timeout) + 1 }')"
  deadline_sec=$((SECONDS + timeout_ceil_sec))

  while (( SECONDS <= deadline_sec )); do
    local current_altitude_m
    current_altitude_m="$(fetch_uav_altitude || true)"
    if is_number "${current_altitude_m}"; then
      LAST_TAKEOFF_WAIT_SAMPLE_COUNT=$((LAST_TAKEOFF_WAIT_SAMPLE_COUNT + 1))
      LAST_TAKEOFF_ALTITUDE_M="${current_altitude_m}"
      LAST_TAKEOFF_ALTITUDE_DELTA_M="$(
        awk -v current="${current_altitude_m}" -v initial="${initial_altitude_m}" \
          'BEGIN { printf "%.3f", current - initial }'
      )"
      if awk -v delta="${LAST_TAKEOFF_ALTITUDE_DELTA_M}" -v required="${required_delta_m}" \
        'BEGIN { exit ((delta + 1.0e-6) >= required) ? 0 : 1 }'
      then
        return 0
      fi
    fi
    sleep 0.2
  done

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
    --takeoff-service)
      if [[ $# -lt 2 ]]; then
        echo "[uav_relative_tracking] missing value for --takeoff-service" >&2
        usage >&2
        exit 1
      fi
      TAKEOFF_SERVICE_NAME="$2"
      shift 2
      ;;
    --takeoff-ready-timeout)
      if [[ $# -lt 2 ]]; then
        echo "[uav_relative_tracking] missing value for --takeoff-ready-timeout" >&2
        usage >&2
        exit 1
      fi
      TAKEOFF_READY_TIMEOUT_SEC="$2"
      shift 2
      ;;
    --takeoff-ready-delta)
      if [[ $# -lt 2 ]]; then
        echo "[uav_relative_tracking] missing value for --takeoff-ready-delta" >&2
        usage >&2
        exit 1
      fi
      TAKEOFF_READY_DELTA_M="$2"
      shift 2
      ;;
    --skip-takeoff)
      USE_TAKEOFF_ON_START="false"
      shift
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
      if is_ignorable_token "$1"; then
        shift
      elif [[ "${ACTION}" == "start" && -z "${SERVICE_NAME}" ]] && is_number "$1"; then
        TARGET_HEIGHT_M="$1"
        shift
      elif [[ -z "${SERVICE_NAME}" ]] && looks_like_ros_name "$1"; then
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

if [[ "${ACTION}" == "start" ]] && is_true "${USE_TAKEOFF_ON_START}" &&
  ! [[ "${TAKEOFF_READY_TIMEOUT_SEC}" =~ ^[0-9]+([.][0-9]+)?$ ]]
then
  echo "[uav_relative_tracking] invalid --takeoff-ready-timeout value: ${TAKEOFF_READY_TIMEOUT_SEC}" >&2
  exit 1
fi

if [[ "${ACTION}" == "start" ]] && is_true "${USE_TAKEOFF_ON_START}" &&
  ! is_number "${TAKEOFF_READY_DELTA_M}"
then
  echo "[uav_relative_tracking] invalid --takeoff-ready-delta value: ${TAKEOFF_READY_DELTA_M}" >&2
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
  log "waiting for relative position fusion readiness on ${DIAGNOSTICS_TOPIC} (timeout=${READY_TIMEOUT_SEC}s)"
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

if [[ "${ACTION}" == "start" ]] && is_true "${USE_TAKEOFF_ON_START}"; then
  local_takeoff_ready_delta_m="$(
    awk -v requested="${TARGET_HEIGHT_M}" -v minimum="${TAKEOFF_READY_DELTA_M}" \
      'BEGIN { if (requested < minimum) { printf "%.3f", requested } else { printf "%.3f", minimum } }'
  )"

  log "waiting for ${TAKEOFF_SERVICE_NAME} (timeout=${WAIT_TIMEOUT_SEC}s)"
  if ! wait_for_service "${TAKEOFF_SERVICE_NAME}" "${WAIT_TIMEOUT_SEC}"; then
    echo "[uav_relative_tracking] takeoff service not available: ${TAKEOFF_SERVICE_NAME}" >&2
    exit 1
  fi

  initial_altitude_m="$(fetch_uav_altitude)"
  if ! is_number "${initial_altitude_m}"; then
    echo "[uav_relative_tracking] failed to read UAV altitude from ${UAV_STATE_TOPIC}" >&2
    exit 1
  fi

  TAKEOFF_SCRIPT="${SCRIPT_DIR}/uav_takeoff.sh"
  log "calling ${TAKEOFF_SERVICE_NAME} via ${TAKEOFF_SCRIPT}"
  if ! TAKEOFF_OUTPUT="$(bash "${TAKEOFF_SCRIPT}" "${TAKEOFF_SERVICE_NAME}" 2>&1)"; then
    printf '%s\n' "${TAKEOFF_OUTPUT}"
    exit 1
  fi
  printf '%s\n' "${TAKEOFF_OUTPUT}"

  if service_response_failed "${TAKEOFF_OUTPUT}"; then
    exit 1
  fi

  log "waiting for UAV climb on ${UAV_STATE_TOPIC} (delta>=${local_takeoff_ready_delta_m}m timeout=${TAKEOFF_READY_TIMEOUT_SEC}s)"
  if ! wait_for_takeoff_climb "${initial_altitude_m}" "${local_takeoff_ready_delta_m}" "${TAKEOFF_READY_TIMEOUT_SEC}"; then
    if (( LAST_TAKEOFF_WAIT_SAMPLE_COUNT == 0 )); then
      echo "[uav_relative_tracking] warning: no readable altitude updates received from ${UAV_STATE_TOPIC} during takeoff wait; continuing" >&2
    else
      echo "[uav_relative_tracking] takeoff climb wait timed out on ${UAV_STATE_TOPIC}" >&2
      if [[ -n "${LAST_TAKEOFF_ALTITUDE_M}" ]]; then
        echo "[uav_relative_tracking] altitude=${LAST_TAKEOFF_ALTITUDE_M} delta=${LAST_TAKEOFF_ALTITUDE_DELTA_M} samples=${LAST_TAKEOFF_WAIT_SAMPLE_COUNT}" >&2
      fi
      exit 1
    fi
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

if [[ "${ACTION}" == "position_mode" ]]; then
  log "waiting for ${POSITION_MODE_CONTROL_SERVICE} (timeout=${WAIT_TIMEOUT_SEC}s)"
  if ! wait_for_service "${POSITION_MODE_CONTROL_SERVICE}" "${WAIT_TIMEOUT_SEC}"; then
    echo "[uav_relative_tracking] position mode control service not available: ${POSITION_MODE_CONTROL_SERVICE}" >&2
    exit 1
  fi

  log "calling ${POSITION_MODE_CONTROL_SERVICE}"
  if ! POSITION_MODE_OUTPUT="$(ros2 service call "${POSITION_MODE_CONTROL_SERVICE}" std_srvs/srv/Trigger "{}" 2>&1)"; then
    printf '%s\n' "${POSITION_MODE_OUTPUT}"
    exit 1
  fi
  printf '%s\n' "${POSITION_MODE_OUTPUT}"

  if service_response_failed "${POSITION_MODE_OUTPUT}"; then
    exit 1
  fi
fi
