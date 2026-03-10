#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SETUP_FILE="${WS_DIR}/install/setup.bash"

if [[ ! -f "${SETUP_FILE}" ]]; then
  echo "[uav_hover] missing ${SETUP_FILE}" >&2
  echo "[uav_hover] build first: cd ${WS_DIR} && colcon build" >&2
  exit 1
fi

# shellcheck disable=SC1090
set +u
source "${SETUP_FILE}"
set -u

SERVICE_NAME="${1:-/uav/control/hover}"
echo "[uav_hover] calling ${SERVICE_NAME}"
ros2 service call "${SERVICE_NAME}" std_srvs/srv/Trigger "{}"
