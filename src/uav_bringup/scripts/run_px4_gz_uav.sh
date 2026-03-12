#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

resolve_first_existing_dir() {
  local candidate=""
  for candidate in "$@"; do
    if [[ -d "${candidate}" ]]; then
      echo "${candidate}"
      return 0
    fi
  done
  return 1
}

INSTALL_PREFIX_CANDIDATE="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SOURCE_PKG_CANDIDATE="$(cd "${SCRIPT_DIR}/.." && pwd)"

if [[ -d "${INSTALL_PREFIX_CANDIDATE}/share/uav_bringup/config/px4/rc" ]]; then
  PKG_SHARE_DIR="${INSTALL_PREFIX_CANDIDATE}/share/uav_bringup"
  WS_DIR="$(cd "${INSTALL_PREFIX_CANDIDATE}/../.." && pwd)"
elif [[ -d "${SOURCE_PKG_CANDIDATE}/config/px4/rc" ]]; then
  PKG_SHARE_DIR="${SOURCE_PKG_CANDIDATE}"
  WS_DIR="$(cd "${SOURCE_PKG_CANDIDATE}/../.." && pwd)"
else
  echo "ERROR [run_px4_gz_uav] Unable to locate uav_bringup config directory." >&2
  echo "ERROR [run_px4_gz_uav] Checked:" >&2
  echo "  - ${INSTALL_PREFIX_CANDIDATE}/share/uav_bringup/config/px4/rc" >&2
  echo "  - ${SOURCE_PKG_CANDIDATE}/config/px4/rc" >&2
  exit 1
fi

LEGACY_REPO_DIR="$(cd "${WS_DIR}/.." && pwd)"
RC_DIR="${PKG_SHARE_DIR}/config/px4/rc"
AIRFRAME_DIR="${PKG_SHARE_DIR}/config/px4/airframes"
MODEL_DIR=""
if ! MODEL_DIR="$(
  resolve_first_existing_dir \
    "${WS_DIR}/install/uav_description/share/uav_description/models" \
    "${WS_DIR}/src/uav_description/models"
)"; then
  echo "ERROR [run_px4_gz_uav] Unable to locate uav_description models directory." >&2
  exit 1
fi

declare -a PX4_CANDIDATES=()
declare -a PX4_TRIED_PATHS=()

append_px4_candidate() {
  local candidate="${1:-}"
  if [[ -z "${candidate}" ]]; then
    return
  fi
  PX4_CANDIDATES+=("${candidate%/}")
}

is_valid_px4_dir() {
  local candidate="$1"
  [[ -f "${candidate}/Makefile" ]] &&
    [[ -f "${candidate}/ROMFS/px4fmu_common/init.d-posix/rcS" ]]
}

resolve_px4_dir() {
  local candidate=""
  local normalized=""
  declare -A seen=()

  for candidate in "${PX4_CANDIDATES[@]}"; do
    normalized="${candidate%/}"
    if [[ -z "${normalized}" ]]; then
      continue
    fi
    if [[ -n "${seen[${normalized}]:-}" ]]; then
      continue
    fi
    seen["${normalized}"]=1
    PX4_TRIED_PATHS+=("${normalized}")
    if is_valid_px4_dir "${normalized}"; then
      echo "${normalized}"
      return 0
    fi
  done

  return 1
}

print_px4_dir_error_and_exit() {
  echo "ERROR [run_px4_gz_uav] Unable to locate PX4-Autopilot directory." >&2
  echo "ERROR [run_px4_gz_uav] Checked paths:" >&2
  if [[ "${#PX4_TRIED_PATHS[@]}" -eq 0 ]]; then
    echo "  - <none>" >&2
  else
    local path=""
    for path in "${PX4_TRIED_PATHS[@]}"; do
      echo "  - ${path}" >&2
    done
  fi
  echo "ERROR [run_px4_gz_uav] Provide one of:" >&2
  echo "  - PX4_DIR=/abs/path/to/PX4-Autopilot" >&2
  echo "  - DUOJIN01_REPO=/abs/path/to/repo_root" >&2
  echo "  - AEROSYMBIO_REPO=/abs/path/to/repo_root" >&2
  exit 1
}

if [[ -n "${PX4_DIR:-}" ]]; then
  PX4_DIR="${PX4_DIR%/}"
  PX4_TRIED_PATHS=("${PX4_DIR}")
  if ! is_valid_px4_dir "${PX4_DIR}"; then
    echo "ERROR [run_px4_gz_uav] PX4_DIR is set but invalid: ${PX4_DIR}" >&2
    print_px4_dir_error_and_exit
  fi
else
  if [[ -n "${DUOJIN01_REPO:-}" ]]; then
    append_px4_candidate "${DUOJIN01_REPO}/third_party/PX4-Autopilot"
  fi
  if [[ -n "${AEROSYMBIO_REPO:-}" ]]; then
    append_px4_candidate "${AEROSYMBIO_REPO}/third_party/PX4-Autopilot"
  fi
  append_px4_candidate "${WS_DIR}/third_party/PX4-Autopilot"
  append_px4_candidate "${LEGACY_REPO_DIR}/third_party/PX4-Autopilot"
  append_px4_candidate "/repo/third_party/PX4-Autopilot"

  if ! PX4_DIR="$(resolve_px4_dir)"; then
    print_px4_dir_error_and_exit
  fi
fi

UAV_PX4_FRAME="${UAV_PX4_FRAME:-hw_uav}"
PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-4901}"
PX4_SIM_MODEL="${PX4_SIM_MODEL:-gz_uav}"
PX4_GZ_WORLD="${PX4_GZ_WORLD:-test}"
PX4_GZ_MODEL_POSE="${PX4_GZ_MODEL_POSE:-1.5,0.0,0.3,0,0,0}"
PX4_INSTANCE="${PX4_INSTANCE:-0}"
HEADLESS="${HEADLESS:-0}"

if ! [[ "${PX4_INSTANCE}" =~ ^[0-9]+$ ]]; then
  echo "ERROR [run_px4_gz_uav] PX4_INSTANCE must be a non-negative integer: ${PX4_INSTANCE}" >&2
  exit 1
fi

RUNTIME_RC_DIR="$(mktemp -d -t uav_px4_rc_XXXXXX)"
trap 'rm -rf "${RUNTIME_RC_DIR}"' EXIT

cp "${RC_DIR}/px4-rc.params" "${RUNTIME_RC_DIR}/px4-rc.params"
FRAME_PARAMS_PATH="${RC_DIR}/frames/${UAV_PX4_FRAME}.params"
if [[ "${UAV_PX4_FRAME}" != "default" ]]; then
  if [[ ! -f "${FRAME_PARAMS_PATH}" ]]; then
    echo "ERROR [run_px4_gz_uav] frame params not found: ${FRAME_PARAMS_PATH}" >&2
    exit 1
  fi
  printf '\n# Frame profile: %s\n' "${UAV_PX4_FRAME}" >> "${RUNTIME_RC_DIR}/px4-rc.params"
  cat "${FRAME_PARAMS_PATH}" >> "${RUNTIME_RC_DIR}/px4-rc.params"
fi

print_runtime_param_summary() {
  local runtime_rc_path="${RUNTIME_RC_DIR}/px4-rc.params"
  local key=""
  local value=""
  local -a keys=(
    "CA_ROTOR0_PX"
    "CA_ROTOR0_PY"
    "CA_ROTOR0_KM"
    "CA_ROTOR1_PX"
    "CA_ROTOR1_PY"
    "CA_ROTOR1_KM"
    "CA_ROTOR2_PX"
    "CA_ROTOR2_PY"
    "CA_ROTOR2_KM"
    "CA_ROTOR3_PX"
    "CA_ROTOR3_PY"
    "CA_ROTOR3_KM"
    "MPC_THR_HOVER"
    "MPC_THR_MIN"
    "MPC_MANTHR_MIN"
    "MPC_Z_VEL_MAX_UP"
    "MPC_Z_V_AUTO_UP"
    "MPC_ACC_UP_MAX"
    "MC_ROLLRATE_P"
    "MC_ROLLRATE_I"
    "MC_ROLLRATE_D"
    "MC_ROLLRATE_FF"
    "MC_PITCHRATE_P"
    "MC_PITCHRATE_I"
    "MC_PITCHRATE_D"
    "MC_PITCHRATE_FF"
    "MC_YAWRATE_P"
    "MC_YAWRATE_I"
    "MC_YAWRATE_D"
    "MC_YAWRATE_MAX"
    "MC_ROLL_P"
    "MC_PITCH_P"
    "MC_YAW_P"
    "MPC_YAWRAUTO_MAX"
    "EKF2_HGT_REF"
    "SENS_EN_GPSSIM"
    "EKF2_GPS_CTRL"
    "SENS_EN_MAGSIM"
    "EKF2_MAG_TYPE"
  )

  echo "INFO  [run_px4_gz_uav] runtime frame profile: ${UAV_PX4_FRAME}"
  echo "INFO  [run_px4_gz_uav] runtime params source: ${runtime_rc_path}"
  for key in "${keys[@]}"; do
    value="$(
      awk -v param_name="${key}" '
        $1 == "param" && $2 == "set" && $3 == param_name {latest = $4}
        END {
          if (latest == "") {
            print "<unset>"
          } else {
            print latest
          }
        }
      ' "${runtime_rc_path}"
    )"
    echo "INFO  [run_px4_gz_uav] ${key}=${value}"
  done
}

print_runtime_param_summary

export PX4_SYS_AUTOSTART
export PX4_SIM_MODEL
export PX4_GZ_WORLD
export PX4_GZ_MODEL_POSE
export HEADLESS
export GZ_SIM_RESOURCE_PATH="${MODEL_DIR}${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
export PATH="${RUNTIME_RC_DIR}:${PATH}"

echo "INFO  [run_px4_gz_uav] using PX4_DIR: ${PX4_DIR}"
echo "INFO  [run_px4_gz_uav] using PX4_INSTANCE: ${PX4_INSTANCE}"

reset_stale_px4_build_dir() {
  local build_dir="${PX4_DIR}/build/px4_sitl_default"
  local cache_file="${build_dir}/CMakeCache.txt"
  local cached_home=""
  local cached_cache_dir=""

  if [[ ! -f "${cache_file}" ]]; then
    return 0
  fi

  cached_home="$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "${cache_file}" | head -n 1)"
  cached_cache_dir="$(sed -n 's/^CMAKE_CACHEFILE_DIR:INTERNAL=//p' "${cache_file}" | head -n 1)"

  if [[ ( -n "${cached_home}" && "${cached_home}" != "${PX4_DIR}" ) ||         ( -n "${cached_cache_dir}" && "${cached_cache_dir}" != "${build_dir}" ) ]]; then
    echo "WARN  [run_px4_gz_uav] stale PX4 build cache detected; resetting ${build_dir}"
    echo "WARN  [run_px4_gz_uav] cached_home=${cached_home}"
    echo "WARN  [run_px4_gz_uav] cached_cache_dir=${cached_cache_dir}"
    rm -rf "${build_dir}"
  fi
}

reset_stale_px4_build_dir

make -C "${PX4_DIR}" px4_sitl

cp "${AIRFRAME_DIR}"/* "${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/"

PX4_BUILD_DIR="${PX4_DIR}/build/px4_sitl_default"
PX4_ROOTFS_DIR="${PX4_BUILD_DIR}/rootfs"
PX4_BIN="${PX4_BUILD_DIR}/bin/px4"

cp "${AIRFRAME_DIR}"/* "${PX4_BUILD_DIR}/etc/init.d-posix/airframes/"
PX4_INSTANCE_ROOTFS_DIR="${PX4_ROOTFS_DIR}/${PX4_INSTANCE}"
mkdir -p "${PX4_INSTANCE_ROOTFS_DIR}"
rm -f \
  "${PX4_ROOTFS_DIR}/parameters.bson" \
  "${PX4_ROOTFS_DIR}/parameters_backup.bson" \
  "${PX4_ROOTFS_DIR}/param_import_fail.bson" \
  "${PX4_INSTANCE_ROOTFS_DIR}/parameters.bson" \
  "${PX4_INSTANCE_ROOTFS_DIR}/parameters_backup.bson" \
  "${PX4_INSTANCE_ROOTFS_DIR}/param_import_fail.bson"

cd "${PX4_ROOTFS_DIR}"
exec "${PX4_BIN}" -i "${PX4_INSTANCE}"
