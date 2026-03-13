#!/usr/bin/env bash
set -euo pipefail

SCRIPT_NAME="$(basename "$0")"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SETUP_FILE="${WS_DIR}/install/setup.bash"
if [[ "${WS_DIR}" != "/ws" && -f "/ws/install/setup.bash" ]]; then
  SETUP_FILE="/ws/install/setup.bash"
fi

DEFAULT_ARRIVAL_TIMEOUT_SEC="${UAV_RELATIVE_GO_ABOVE_WAIT_TIMEOUT_SEC:-8}"
DEFAULT_STABLE_SEC="${UAV_RELATIVE_GO_ABOVE_STABLE_SEC:-0.6}"
DEFAULT_XY_TOLERANCE_M="${UAV_RELATIVE_GO_ABOVE_XY_TOLERANCE_M:-0.25}"
DEFAULT_Z_TOLERANCE_M="${UAV_RELATIVE_GO_ABOVE_Z_TOLERANCE_M:-0.25}"
DEFAULT_VL_WAIT_TIMEOUT_SEC="${UAV_RELATIVE_GO_ABOVE_VL_WAIT_TIMEOUT_SEC:-5}"
RELATIVE_POSE_TOPIC="${UAV_RELATIVE_GO_ABOVE_RELATIVE_POSE_TOPIC:-/relative_position/estimate/uav_body}"
UAV_STATE_TOPIC="${UAV_RELATIVE_GO_ABOVE_UAV_STATE_TOPIC:-/uav/state/odometry}"
UGV_ODOM_TOPIC="${UAV_RELATIVE_GO_ABOVE_UGV_ODOM_TOPIC:-/ugv/odom}"
RELATIVE_TRACKING_CONFIG_FILE="${UAV_RELATIVE_GO_ABOVE_CONFIG_FILE:-${WS_DIR}/src/relative_position_fusion/config/relative_tracking.common.yaml}"

ARRIVAL_TIMEOUT_SEC="${DEFAULT_ARRIVAL_TIMEOUT_SEC}"
STABLE_SEC="${DEFAULT_STABLE_SEC}"
XY_TOLERANCE_M="${DEFAULT_XY_TOLERANCE_M}"
Z_TOLERANCE_M="${DEFAULT_Z_TOLERANCE_M}"
VL_WAIT_TIMEOUT_SEC="${DEFAULT_VL_WAIT_TIMEOUT_SEC}"
WAIT_FOR_ARRIVAL="true"
RETURN_HEIGHT_M=""
FORWARDED_ARGS=()

LAST_PLANAR_ERROR_M=""
LAST_HEIGHT_ERROR_M=""
LAST_RELATIVE_X_M=""
LAST_RELATIVE_Y_M=""
LAST_UAV_HEIGHT_M=""
LAST_UGV_HEIGHT_M=""

usage() {
  cat <<EOF_USAGE
Usage:
  ${SCRIPT_NAME} [go_above_ugv args...] [--arrival-timeout SEC] [--stable SEC] [--xy-tolerance M] [--z-tolerance M] [--return-height M] [--vl-timeout SEC] [--no-arrival-wait]

Behavior:
  1. Call ./ws/scripts/uav_relative_tracking.sh go_above_ugv
  2. Optionally wait until the UAV is inferred to be above the UGV
  3. Stop relative tracking
  4. Start visual landing

Options:
  --arrival-timeout SEC  Wait up to SEC seconds for the above-UGV condition.
  --stable SEC           Require the condition to hold for SEC seconds.
  --xy-tolerance M       Max planar relative error to treat as above UGV.
  --z-tolerance M        Max height error around return_height_m.
  --return-height M      Override the expected go_above_ugv target height.
  --vl-timeout SEC       Wait timeout passed to uav_visual_landing.sh start.
  --no-arrival-wait      Skip the above-UGV check and hand off immediately after go_above_ugv succeeds.
  -h, --help             Show this help.

Notes:
  - Extra arguments are forwarded to uav_relative_tracking.sh go_above_ugv.
  - The default return height is loaded from:
    ${RELATIVE_TRACKING_CONFIG_FILE}
  - After handoff, relative tracking is stopped and will not be resumed by this script.

Examples:
  ./ws/scripts/uav_relative_go_above_ugv.sh
  ./ws/scripts/uav_relative_go_above_ugv.sh --arrival-timeout 10
  ./ws/scripts/uav_relative_go_above_ugv.sh --xy-tolerance 0.20 --z-tolerance 0.15
  ./ws/scripts/uav_relative_go_above_ugv.sh --no-arrival-wait
EOF_USAGE
}

log() {
  echo "[uav_relative_go_above_ugv] $*"
}

service_output_failed() {
  grep -Eq 'success(=|:[[:space:]]*)(False|false)' <<<"$1"
}

is_number() {
  [[ "$1" =~ ^[-+]?[0-9]+([.][0-9]+)?$ ]]
}

is_ignorable_token() {
  [[ "$1" =~ ^[[:space:][:punct:]，。！？；：、（）【】《》￥…“”‘’]+$ ]]
}

load_return_height_from_config() {
  local config_file="$1"
  if [[ ! -f "${config_file}" ]]; then
    return 1
  fi

  awk '
    /^[[:space:]]*return_height_m:[[:space:]]*/ {
      value = $0
      sub(/^[[:space:]]*return_height_m:[[:space:]]*/, "", value)
      sub(/[[:space:]]*(#.*)?$/, "", value)
      print value
      exit
    }' "${config_file}"
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

fetch_topic_field() {
  local topic_name="$1"
  local topic_type="$2"
  local field_name="$3"

  timeout 1s ros2 topic echo \
    --once \
    --qos-profile sensor_data \
    --field "${field_name}" \
    "${topic_name}" \
    "${topic_type}" 2>/dev/null | extract_first_number || true
}

now_monotonic_sec() {
  date +%s.%N
}

wait_for_above_ugv() {
  local target_height_m="$1"
  local xy_tolerance_m="$2"
  local z_tolerance_m="$3"
  local stable_sec="$4"
  local timeout_sec="$5"
  local timeout_ceil_sec
  local deadline_sec
  local stable_since_sec=""

  timeout_ceil_sec="$(awk -v timeout="${timeout_sec}" 'BEGIN { print (timeout == int(timeout)) ? int(timeout) : int(timeout) + 1 }')"
  deadline_sec=$((SECONDS + timeout_ceil_sec))

  while (( SECONDS <= deadline_sec )); do
    local rel_x_m
    local rel_y_m
    local uav_height_m
    local ugv_height_m

    rel_x_m="$(fetch_topic_field "${RELATIVE_POSE_TOPIC}" geometry_msgs/msg/PoseWithCovarianceStamped pose.pose.position.x)"
    rel_y_m="$(fetch_topic_field "${RELATIVE_POSE_TOPIC}" geometry_msgs/msg/PoseWithCovarianceStamped pose.pose.position.y)"
    uav_height_m="$(fetch_topic_field "${UAV_STATE_TOPIC}" nav_msgs/msg/Odometry pose.pose.position.z)"
    ugv_height_m="$(fetch_topic_field "${UGV_ODOM_TOPIC}" nav_msgs/msg/Odometry pose.pose.position.z)"

    if is_number "${rel_x_m}" && is_number "${rel_y_m}" &&
      is_number "${uav_height_m}" && is_number "${ugv_height_m}"
    then
      LAST_RELATIVE_X_M="${rel_x_m}"
      LAST_RELATIVE_Y_M="${rel_y_m}"
      LAST_UAV_HEIGHT_M="${uav_height_m}"
      LAST_UGV_HEIGHT_M="${ugv_height_m}"
      LAST_PLANAR_ERROR_M="$(
        awk -v x="${rel_x_m}" -v y="${rel_y_m}" 'BEGIN { printf "%.3f", sqrt((x * x) + (y * y)) }'
      )"
      LAST_HEIGHT_ERROR_M="$(
        awk -v uav="${uav_height_m}" -v ugv="${ugv_height_m}" -v target="${target_height_m}" \
          'BEGIN {
             diff = (uav - ugv) - target
             if (diff < 0) {
               diff = -diff
             }
             printf "%.3f", diff
           }'
      )"

      if awk -v planar="${LAST_PLANAR_ERROR_M}" -v planar_tol="${xy_tolerance_m}" \
          -v z_err="${LAST_HEIGHT_ERROR_M}" -v z_tol="${z_tolerance_m}" \
          'BEGIN { exit ((planar <= planar_tol + 1.0e-6) && (z_err <= z_tol + 1.0e-6)) ? 0 : 1 }'
      then
        if [[ -z "${stable_since_sec}" ]]; then
          stable_since_sec="$(now_monotonic_sec)"
        elif awk -v now="$(now_monotonic_sec)" -v since="${stable_since_sec}" -v stable="${stable_sec}" \
          'BEGIN { exit (((now - since) + 1.0e-6) >= stable) ? 0 : 1 }'
        then
          return 0
        fi
      else
        stable_since_sec=""
      fi
    fi

    sleep 0.2
  done

  return 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --arrival-timeout)
      if [[ $# -lt 2 ]]; then
        echo "[uav_relative_go_above_ugv] missing value for --arrival-timeout" >&2
        usage >&2
        exit 1
      fi
      ARRIVAL_TIMEOUT_SEC="$2"
      shift 2
      ;;
    --stable)
      if [[ $# -lt 2 ]]; then
        echo "[uav_relative_go_above_ugv] missing value for --stable" >&2
        usage >&2
        exit 1
      fi
      STABLE_SEC="$2"
      shift 2
      ;;
    --xy-tolerance)
      if [[ $# -lt 2 ]]; then
        echo "[uav_relative_go_above_ugv] missing value for --xy-tolerance" >&2
        usage >&2
        exit 1
      fi
      XY_TOLERANCE_M="$2"
      shift 2
      ;;
    --z-tolerance)
      if [[ $# -lt 2 ]]; then
        echo "[uav_relative_go_above_ugv] missing value for --z-tolerance" >&2
        usage >&2
        exit 1
      fi
      Z_TOLERANCE_M="$2"
      shift 2
      ;;
    --return-height)
      if [[ $# -lt 2 ]]; then
        echo "[uav_relative_go_above_ugv] missing value for --return-height" >&2
        usage >&2
        exit 1
      fi
      RETURN_HEIGHT_M="$2"
      shift 2
      ;;
    --vl-timeout)
      if [[ $# -lt 2 ]]; then
        echo "[uav_relative_go_above_ugv] missing value for --vl-timeout" >&2
        usage >&2
        exit 1
      fi
      VL_WAIT_TIMEOUT_SEC="$2"
      shift 2
      ;;
    --no-arrival-wait)
      WAIT_FOR_ARRIVAL="false"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      if is_ignorable_token "$1"; then
        log "ignoring stray token: $1"
      else
        FORWARDED_ARGS+=("$1")
      fi
      shift
      ;;
  esac
done

if [[ -z "${RETURN_HEIGHT_M}" ]]; then
  RETURN_HEIGHT_M="$(load_return_height_from_config "${RELATIVE_TRACKING_CONFIG_FILE}" || true)"
fi
RETURN_HEIGHT_M="${RETURN_HEIGHT_M:-1.0}"

for numeric_value in "${ARRIVAL_TIMEOUT_SEC}" "${STABLE_SEC}" "${XY_TOLERANCE_M}" "${Z_TOLERANCE_M}" "${VL_WAIT_TIMEOUT_SEC}" "${RETURN_HEIGHT_M}"; do
  if ! is_number "${numeric_value}"; then
    echo "[uav_relative_go_above_ugv] invalid numeric value: ${numeric_value}" >&2
    exit 1
  fi
done

if [[ ! -f "${SETUP_FILE}" ]]; then
  echo "[uav_relative_go_above_ugv] missing ${SETUP_FILE}" >&2
  echo "[uav_relative_go_above_ugv] build first: cd ${WS_DIR} && colcon build" >&2
  exit 1
fi

set +u
# shellcheck source=/dev/null
source "${SETUP_FILE}"
set -u

GO_ABOVE_SCRIPT="${SCRIPT_DIR}/uav_relative_tracking.sh"
START_SCRIPT="${SCRIPT_DIR}/uav_relative_start.sh"
VISUAL_LANDING_SCRIPT="${SCRIPT_DIR}/uav_visual_landing.sh"

log "calling go_above_ugv"
if ! GO_ABOVE_OUTPUT="$(bash "${GO_ABOVE_SCRIPT}" go_above_ugv "${FORWARDED_ARGS[@]}" 2>&1)"; then
  if grep -Fq "relative tracking has not been started" <<<"${GO_ABOVE_OUTPUT}"; then
    log "relative tracking session is inactive; starting a new session at ${RETURN_HEIGHT_M}m before handoff"
    if ! START_OUTPUT="$(bash "${START_SCRIPT}" "${RETURN_HEIGHT_M}" 2>&1)"; then
      printf '%s\n' "${GO_ABOVE_OUTPUT}"
      printf '%s\n' "${START_OUTPUT}"
      exit 1
    fi
    printf '%s\n' "${START_OUTPUT}"
  else
    printf '%s\n' "${GO_ABOVE_OUTPUT}"
    exit 1
  fi
elif service_output_failed "${GO_ABOVE_OUTPUT}"; then
  if grep -Fq "relative tracking has not been started" <<<"${GO_ABOVE_OUTPUT}"; then
    log "relative tracking session is inactive; starting a new session at ${RETURN_HEIGHT_M}m before handoff"
    if ! START_OUTPUT="$(bash "${START_SCRIPT}" "${RETURN_HEIGHT_M}" 2>&1)"; then
      printf '%s\n' "${GO_ABOVE_OUTPUT}"
      printf '%s\n' "${START_OUTPUT}"
      exit 1
    fi
    printf '%s\n' "${START_OUTPUT}"
  else
    printf '%s\n' "${GO_ABOVE_OUTPUT}"
    exit 1
  fi
else
  printf '%s\n' "${GO_ABOVE_OUTPUT}"
fi

if [[ "${WAIT_FOR_ARRIVAL}" == "true" ]]; then
  log "waiting until UAV is above UGV (xy<=${XY_TOLERANCE_M}m z_err<=${Z_TOLERANCE_M}m stable=${STABLE_SEC}s timeout=${ARRIVAL_TIMEOUT_SEC}s target_height=${RETURN_HEIGHT_M}m)"
  if wait_for_above_ugv "${RETURN_HEIGHT_M}" "${XY_TOLERANCE_M}" "${Z_TOLERANCE_M}" "${STABLE_SEC}" "${ARRIVAL_TIMEOUT_SEC}"; then
    log "above-UGV handoff condition met (planar=${LAST_PLANAR_ERROR_M}m height_err=${LAST_HEIGHT_ERROR_M}m)"
  else
    log "above-UGV handoff condition not confirmed in time; proceeding after go_above_ugv success"
    if [[ -n "${LAST_PLANAR_ERROR_M}" && -n "${LAST_HEIGHT_ERROR_M}" ]]; then
      log "last observed planar=${LAST_PLANAR_ERROR_M}m height_err=${LAST_HEIGHT_ERROR_M}m rel=(${LAST_RELATIVE_X_M},${LAST_RELATIVE_Y_M}) z=(${LAST_UAV_HEIGHT_M},${LAST_UGV_HEIGHT_M})"
    fi
  fi
fi

log "stopping relative tracking before visual landing handoff"
bash "${GO_ABOVE_SCRIPT}" stop

log "starting visual landing"
bash "${VISUAL_LANDING_SCRIPT}" start --timeout "${VL_WAIT_TIMEOUT_SEC}"

log "visual landing owns control from here; relative tracking will not resume after tag detection or loss"
