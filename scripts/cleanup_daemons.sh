#!/usr/bin/env bash
set -euo pipefail

SCRIPT_NAME="$(basename "$0")"
TMP_ROOT="${TMPDIR:-/tmp}"

DRY_RUN=0
FORCE_KILL=0
STOP_ROS2_DAEMON=1
GRACE_TIMEOUT_SEC="${CLEANUP_GRACE_TIMEOUT_SEC:-4}"
WAIT_INTERVAL_SEC="0.2"
MAX_PASSES=2

declare -A PROTECTED_PIDS=()
declare -A TARGET_LABELS=()
declare -A TARGET_DEPTHS=()
declare -a TARGET_PID_LIST=()

PROCESS_RULES=(
  "ros2 daemon|[r]os2 daemon"
  "duojin sim launch|([r]os2 launch .*duojin01_bringup .*sim\\.launch\\.py|duojin01_bringup.*/launch/sim\\.launch\\.py)"
  "duojin sim_navigation launch|([r]os2 launch .*duojin01_bringup .*sim_navigation\\.launch\\.py|duojin01_bringup.*/launch/sim_navigation\\.launch\\.py)"
  "uav sitl launch|([r]os2 launch .*uav_bringup .*sitl_uav\\.launch\\.py|uav_bringup.*/launch/sitl_uav\\.launch\\.py)"
  "uav gz launch|([r]os2 launch .*uav_bringup .*gz_sim\\.launch\\.py|uav_bringup.*/launch/gz_sim\\.launch\\.py)"
  "ugv sim launch|([r]os2 launch .*ugv_bringup .*sim\\.launch\\.py|ugv_bringup.*/launch/sim\\.launch\\.py)"
  "ugv sim_navigation launch|([r]os2 launch .*ugv_bringup .*sim_navigation\\.launch\\.py|ugv_bringup.*/launch/sim_navigation\\.launch\\.py)"
  "ugv nav2 launch|([r]os2 launch .*ugv_bringup .*nav2\\.launch\\.py|ugv_bringup.*/launch/nav2\\.launch\\.py)"
  "sim clock launch|([r]os2 launch .*sim_worlds .*sim_clock\\.launch\\.py|sim_worlds.*/launch/sim_clock\\.launch\\.py)"
  "run_px4_gz_uav.sh|[r]un_px4_gz_uav\\.sh"
  "MicroXRCEAgent|[M]icroXRCEAgent"
  "PX4 SITL|(^|[/ ])px4([ ]|$)"
  "Gazebo Sim|[g]z sim"
  "Gazebo Server|[g]z-server"
  "Gazebo GUI|[g]z-gui"
  "Gazebo Client|[g]z-client"
  "sim_clock_bridge|__node:=sim_clock_bridge"
  "ros_gz_bridge|__node:=ros_gz_bridge"
  "spawn_ugv_model|__node:=spawn_ugv_model"
  "gz_ground_truth_bridge|[t]f_bridge_node|__node:=gz_ground_truth_bridge"
  "uav_state_bridge|[u]av_state_bridge_node|__node:=uav_state_bridge"
  "px4_planar_state_reader|[p]x4_planar_state_reader_node|__node:=px4_planar_state_reader"
  "mono_camera_bridge|[m]ono_camera_bridge_node|__node:=mono_camera_bridge"
  "stereo_camera_bridge|[s]tereo_camera_bridge_node|__node:=stereo_camera_bridge"
  "fmu_topic_namespace_bridge|[f]mu_topic_namespace_bridge_node|__node:=fmu_topic_namespace_bridge"
  "uav_control|[u]av_control_node|__node:=uav_control"
  "rangefinder_bridge|[r]angefinder_bridge_node|__node:=rangefinder_bridge"
  "optical_flow_bridge|[o]ptical_flow_bridge_node|__node:=optical_flow_bridge"
  "aruco_detector|[a]ruco_detector_node|__node:=aruco_detector_node"
  "visual_landing|[v]isual_landing_node|__node:=visual_landing_node"
  "uav_robot_state_publisher|__node:=uav_robot_state_publisher"
  "uav_ground_truth_robot_state_publisher|__node:=uav_ground_truth_robot_state_publisher"
  "global_to_uav_map_tf|__node:=global_to_uav_map_tf"
  "relative_position_fuser|[r]elative_position_fusion_node|__node:=relative_position_fuser"
  "ugv robot_state_publisher|[/]ugv/robot_description|[/]ugv/joint_states"
  "joint_state_stamp_fix|[j]oint_state_stamp_fix_node|__node:=joint_state_stamp_fix"
  "ugv_controller_emulator|[u]gv_controller_emulator_node|__node:=ugv_controller_emulator"
  "sim_odom_to_tf|[o]dom_to_tf_node|__node:=sim_odom_to_tf"
  "map_to_ugv_odom_tf|__node:=map_to_ugv_odom_tf"
  "global_to_ugv_map_tf|__node:=global_to_ugv_map_tf"
  "scan_frame_rewriter|[s]can_frame_rewriter|__node:=scan_frame_rewriter"
  "nav2_container|component_container(_isolated|_mt)? .*__node:=nav2_container|__node:=nav2_container"
  "map_server|__node:=map_server"
  "amcl|__node:=amcl"
  "planner_server|__node:=planner_server"
  "controller_server|__node:=controller_server"
  "behavior_server|__node:=behavior_server"
  "bt_navigator|__node:=bt_navigator"
  "waypoint_follower|__node:=waypoint_follower"
  "velocity_smoother|__node:=velocity_smoother"
  "smoother_server|__node:=smoother_server"
  "lifecycle_manager_navigation|__node:=lifecycle_manager_navigation"
  "lifecycle_manager_localization|__node:=lifecycle_manager_localization"
  "initial_pose_publisher|[i]nitial_pose_publisher|__node:=initial_pose_publisher"
  "orbbec_topic_compat|[o]rbbec_topic_compat_node|__node:=orbbec_topic_compat"
  "twist_mux|__node:=twist_mux"
  "ugv_base_driver|[u]gv_base_driver_node|__node:=ugv_base_driver"
  "safety_watchdog|[s]afety_watchdog_node|__node:=safety_watchdog"
  "ekf_filter_node|__node:=ekf_filter_node"
  "foxglove_bridge|[f]oxglove_bridge([ ]|$)|__node:=foxglove_bridge_node"
  "joy_node|__node:=joy_node"
  "joy_axis_selector|[j]oy_axis_selector_node|__node:=joy_axis_selector_node"
  "joy_launcher|[j]oy_launcher_node|__node:=joy_launcher_node"
  "teleop_slow|__node:=teleop_slow"
  "teleop_normal|__node:=teleop_normal"
  "teleop_estop|__node:=teleop_estop"
  "keyboard_gui_teleop|[k]eyboard_gui_teleop_node|__node:=keyboard_gui_teleop_node"
  "keyboard_tty_teleop|[k]eyboard_tty_teleop_node|__node:=keyboard_tty_teleop_node"
  "keyboard_teleop|[k]eyboard_teleop_node|__node:=keyboard_teleop_node"
  "stack rviz|[r]viz2 .*(duojin01_bringup|uav_bringup|ugv_bringup).*/config/rviz/.*\\.rviz"
)

SOCKET_RULES=(
  "MicroXRCEAgent UDP port 8888|8888"
)

TEMP_PATH_GLOBS=(
  "${TMP_ROOT}/uav_px4_rc_*"
  "${TMP_ROOT}/ugv_sim_model_*"
)

usage() {
  cat <<EOF
Usage: ${SCRIPT_NAME} [--dry-run] [--force] [--keep-ros2-daemon] [--grace-timeout SEC]

Clean lingering processes and temp artifacts from the current DUOJIN/UAV/UGV sim stack:
  - ros2 launch roots and their descendant process trees
  - PX4 SITL, MicroXRCEAgent, Gazebo sim/server/gui/client
  - ros_gz_bridge, UAV bridge nodes, visual landing nodes, RViz
  - UGV sim nodes, teleop helpers, relative_position_fusion
  - stale temp dirs: ${TMP_ROOT}/uav_px4_rc_* and ${TMP_ROOT}/ugv_sim_model_*

Options:
  --dry-run           Print what would be stopped/removed without changing anything.
  --force             Skip graceful shutdown and send SIGKILL directly.
  --keep-ros2-daemon  Do not stop ros2 daemon.
  --grace-timeout SEC Wait up to SEC seconds after SIGTERM before SIGKILL.
  -h, --help          Show this help.
EOF
}

log() {
  echo "[${SCRIPT_NAME}] $*"
}

warn() {
  echo "[${SCRIPT_NAME}] WARN: $*" >&2
}

have_cmd() {
  command -v "$1" >/dev/null 2>&1
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
    --grace-timeout)
      if [[ $# -lt 2 ]]; then
        echo "[${SCRIPT_NAME}] missing value for --grace-timeout" >&2
        usage >&2
        exit 1
      fi
      GRACE_TIMEOUT_SEC="$2"
      shift
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

if ! [[ "${GRACE_TIMEOUT_SEC}" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "[${SCRIPT_NAME}] invalid --grace-timeout value: ${GRACE_TIMEOUT_SEC}" >&2
  exit 1
fi

GRACE_LOOPS="$(
  awk -v timeout="${GRACE_TIMEOUT_SEC}" -v interval="${WAIT_INTERVAL_SEC}" '
    BEGIN {
      loops = int((timeout / interval) + 0.999999)
      if (loops < 1) {
        loops = 1
      }
      print loops
    }
  '
)"

process_state() {
  local pid="$1"
  ps -p "${pid}" -o stat= 2>/dev/null | awk '{print substr($1, 1, 1)}'
}

process_ppid() {
  local pid="$1"
  ps -p "${pid}" -o ppid= 2>/dev/null | awk '{print $1}'
}

process_args() {
  local pid="$1"
  ps -p "${pid}" -o args= 2>/dev/null || true
}

build_protected_pid_set() {
  local pid="$$"
  local ppid=""

  while [[ -n "${pid}" && "${pid}" =~ ^[0-9]+$ ]]; do
    if [[ "${pid}" -le 1 ]]; then
      break
    fi

    PROTECTED_PIDS["${pid}"]=1
    ppid="$(process_ppid "${pid}")"
    if [[ -z "${ppid}" || "${ppid}" == "${pid}" ]]; then
      break
    fi
    pid="${ppid}"
  done
}

is_protected_pid() {
  local pid="$1"
  [[ -n "${PROTECTED_PIDS[${pid}]:-}" ]]
}

append_label() {
  local pid="$1"
  local label="$2"
  local existing="${TARGET_LABELS[${pid}]:-}"

  if [[ -z "${existing}" ]]; then
    TARGET_LABELS["${pid}"]="${label}"
    return 0
  fi

  case ",${existing}," in
    *",${label},"*)
      ;;
    *)
      TARGET_LABELS["${pid}"]="${existing}, ${label}"
      ;;
  esac
}

add_target_pid() {
  local pid="$1"
  local label="$2"
  local depth="$3"

  [[ "${pid}" =~ ^[0-9]+$ ]] || return 0
  is_protected_pid "${pid}" && return 0

  if [[ -z "${TARGET_DEPTHS[${pid}]:-}" ]]; then
    TARGET_PID_LIST+=("${pid}")
    TARGET_DEPTHS["${pid}"]="${depth}"
    TARGET_LABELS["${pid}"]="${label}"
    return 0
  fi

  if (( depth > TARGET_DEPTHS["${pid}"] )); then
    TARGET_DEPTHS["${pid}"]="${depth}"
  fi
  append_label "${pid}" "${label}"
}

collect_pid_tree() {
  local root_pid="$1"
  local label="$2"
  local current_pid=""
  local current_depth=""
  local -a queue_pids=("${root_pid}")
  local -a queue_depths=(0)
  local -a children=()
  declare -A seen=()

  while (( ${#queue_pids[@]} > 0 )); do
    current_pid="${queue_pids[0]}"
    current_depth="${queue_depths[0]}"
    queue_pids=("${queue_pids[@]:1}")
    queue_depths=("${queue_depths[@]:1}")

    if [[ -n "${seen[${current_pid}]:-}" ]]; then
      continue
    fi
    seen["${current_pid}"]=1

    add_target_pid "${current_pid}" "${label}" "${current_depth}"

    mapfile -t children < <(pgrep -P "${current_pid}" 2>/dev/null || true)
    if (( ${#children[@]} == 0 )); then
      continue
    fi

    for child_pid in "${children[@]}"; do
      [[ -z "${child_pid}" ]] && continue
      queue_pids+=("${child_pid}")
      queue_depths+=("$((current_depth + 1))")
    done
  done
}

scan_rule() {
  local label="$1"
  local pattern="$2"
  local -a pids=()

  if [[ "${label}" == "ros2 daemon" && "${STOP_ROS2_DAEMON}" -eq 0 ]]; then
    return 0
  fi

  mapfile -t pids < <(pgrep -f "${pattern}" 2>/dev/null || true)
  for pid in "${pids[@]}"; do
    [[ -z "${pid}" ]] && continue
    collect_pid_tree "${pid}" "${label}"
  done
}

scan_socket_rule() {
  local label="$1"
  local port="$2"
  local -a pids=()

  have_cmd ss || return 0

  mapfile -t pids < <(
    ss -H -lunp 2>/dev/null | awk -v port=":${port}" '
      $5 ~ port"$" {
        while (match($0, /pid=[0-9]+/)) {
          print substr($0, RSTART + 4, RLENGTH - 4)
          $0 = substr($0, RSTART + RLENGTH)
        }
      }
    ' | sort -u
  )

  for pid in "${pids[@]}"; do
    [[ -z "${pid}" ]] && continue
    collect_pid_tree "${pid}" "${label}"
  done
}

reset_targets() {
  TARGET_LABELS=()
  TARGET_DEPTHS=()
  TARGET_PID_LIST=()
}

scan_all_targets() {
  local entry=""
  local label=""
  local pattern=""

  reset_targets

  for entry in "${PROCESS_RULES[@]}"; do
    label="${entry%%|*}"
    pattern="${entry#*|}"
    scan_rule "${label}" "${pattern}"
  done

  for entry in "${SOCKET_RULES[@]}"; do
    label="${entry%%|*}"
    pattern="${entry#*|}"
    scan_socket_rule "${label}" "${pattern}"
  done
}

sorted_target_pids() {
  local pid=""
  for pid in "${TARGET_PID_LIST[@]}"; do
    printf '%s\t%s\n' "${TARGET_DEPTHS[${pid}]}" "${pid}"
  done | sort -k1,1nr -k2,2n | awk '{print $2}'
}

print_target() {
  local pid="$1"
  local state=""
  local ppid=""
  local args=""

  state="$(process_state "${pid}")"
  ppid="$(process_ppid "${pid}")"
  args="$(process_args "${pid}")"
  if [[ -z "${args}" ]]; then
    args="<exited>"
  fi

  log "matched ${TARGET_LABELS[${pid}]}: pid=${pid} depth=${TARGET_DEPTHS[${pid}]} ppid=${ppid:-?} state=${state:-?} cmd=${args}"
}

list_targets() {
  local pid=""

  if (( ${#TARGET_PID_LIST[@]} == 0 )); then
    log "no matching processes found"
    return 0
  fi

  while IFS= read -r pid; do
    [[ -z "${pid}" ]] && continue
    print_target "${pid}"
  done < <(sorted_target_pids)
}

pid_is_effectively_gone() {
  local pid="$1"
  local state=""

  if ! kill -0 "${pid}" 2>/dev/null; then
    return 0
  fi

  state="$(process_state "${pid}")"
  [[ "${state}" == "Z" ]]
}

stop_pid() {
  local pid="$1"
  local labels="$2"
  local depth="$3"
  local state=""
  local ppid=""
  local args=""

  if pid_is_effectively_gone "${pid}"; then
    state="$(process_state "${pid}")"
    if [[ "${state}" == "Z" ]]; then
      ppid="$(process_ppid "${pid}")"
      log "zombie ${labels}: pid=${pid} depth=${depth} ppid=${ppid:-?} (cannot signal zombie directly)"
    fi
    return 0
  fi

  state="$(process_state "${pid}")"
  args="$(process_args "${pid}")"
  if [[ -z "${args}" ]]; then
    args="<exited>"
  fi

  if (( DRY_RUN )); then
    log "would stop ${labels}: pid=${pid} depth=${depth} state=${state:-?} cmd=${args}"
    return 0
  fi

  if (( FORCE_KILL )); then
    log "SIGKILL ${labels}: pid=${pid} depth=${depth}"
    kill -KILL "${pid}" 2>/dev/null || true
    return 0
  fi

  log "SIGTERM ${labels}: pid=${pid} depth=${depth}"
  kill -TERM "${pid}" 2>/dev/null || true

  for _ in $(seq 1 "${GRACE_LOOPS}"); do
    if pid_is_effectively_gone "${pid}"; then
      return 0
    fi
    sleep "${WAIT_INTERVAL_SEC}"
  done

  log "SIGKILL ${labels}: pid=${pid} depth=${depth}"
  kill -KILL "${pid}" 2>/dev/null || true
}

stop_targets() {
  local pid=""

  while IFS= read -r pid; do
    [[ -z "${pid}" ]] && continue
    stop_pid "${pid}" "${TARGET_LABELS[${pid}]}" "${TARGET_DEPTHS[${pid}]}"
  done < <(sorted_target_pids)
}

cleanup_temp_paths() {
  local pattern=""
  local path=""
  local found=0

  for pattern in "${TEMP_PATH_GLOBS[@]}"; do
    while IFS= read -r path; do
      [[ -z "${path}" ]] && continue
      found=1
      if (( DRY_RUN )); then
        log "would remove temp path: ${path}"
      else
        log "removing temp path: ${path}"
        rm -rf -- "${path}"
      fi
    done < <(compgen -G "${pattern}" || true)
  done

  if [[ "${found}" -eq 0 ]]; then
    log "no matching temp paths found under ${TMP_ROOT}"
  fi
}

build_protected_pid_set

if [[ "${STOP_ROS2_DAEMON}" -eq 1 ]]; then
  if (( DRY_RUN )); then
    log "would request ros2 daemon stop"
  elif have_cmd ros2; then
    log "requesting ros2 daemon stop"
    ros2 daemon stop >/dev/null 2>&1 || true
  fi
fi

if (( DRY_RUN )); then
  scan_all_targets
  list_targets
  cleanup_temp_paths
  log "dry-run complete"
  exit 0
fi

for pass in $(seq 1 "${MAX_PASSES}"); do
  scan_all_targets
  if (( ${#TARGET_PID_LIST[@]} == 0 )); then
    log "cleanup pass ${pass}: no matching processes left"
    break
  fi

  log "cleanup pass ${pass}: stopping ${#TARGET_PID_LIST[@]} process(es)"
  stop_targets
done

cleanup_temp_paths

scan_all_targets
if (( ${#TARGET_PID_LIST[@]} > 0 )); then
  warn "cleanup finished with ${#TARGET_PID_LIST[@]} matching process(es) still present"
  list_targets
else
  log "cleanup complete"
fi
