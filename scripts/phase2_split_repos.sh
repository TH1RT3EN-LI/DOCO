#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  phase2_split_repos.sh [--output-dir DIR] [--execute]

Default mode is dry-run: print the clone + git-filter-repo commands without executing them.

Examples:
  ./scripts/phase2_split_repos.sh
  ./scripts/phase2_split_repos.sh --output-dir ~/tmp/doco-split --execute
EOF
}

OUTPUT_DIR="${HOME}/tmp/doco-split"
EXECUTE=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --execute)
      EXECUTE=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

REPO_ROOT="$(git rev-parse --show-toplevel)"
REPO_URL="$(git -C "$REPO_ROOT" remote get-url origin 2>/dev/null || true)"
if [[ -z "$REPO_URL" ]]; then
  echo "ERROR: unable to resolve origin remote for $REPO_ROOT" >&2
  exit 1
fi

if ! command -v git-filter-repo >/dev/null 2>&1; then
  echo "ERROR: git-filter-repo is required for phase 2 splitting." >&2
  echo "Install it first, then rerun this script." >&2
  exit 1
fi

SPECS=$(cat <<'EOF'
doco_common|src/quadrotor_msgs src/relative_position_fusion README.relative_position_fusion.architecture.md README.relative_position_fusion.codex.md README.relative_position_fusion.implementation.md README.relative_position_fusion.interfaces.md README.relative_position_fusion.verification.md REPORT.relative_position_fusion.md
doco_uav|src/uav_bridge src/uav_bringup src/uav_visual_landing src/uav_mode_supervisor src/uav_description
doco_ugv|src/ugv_base_driver src/ugv_bringup src/ugv_description src/ugv_safety_watchdog src/ugv_slam_tools src/ugv_teleop src/lslidar_driver
doco_sim|src/uav_gz_bridge src/uav_sim_bringup src/ugv_sim_bringup src/ugv_controller_emulator src/ugv_sim_tools src/sim_worlds src/duojin01_bringup
EOF
)

MANIFESTS_SPEC='doco_manifests|manifests'

mkdir -p "$OUTPUT_DIR"

echo "Phase 2 split plan"
echo "- source repo: $REPO_ROOT"
echo "- origin url:  $REPO_URL"
echo "- output dir:  $OUTPUT_DIR"
echo "- mode:        $([[ $EXECUTE -eq 1 ]] && echo execute || echo dry-run)"
echo

run_or_print() {
  if [[ $EXECUTE -eq 1 ]]; then
    echo "+ $*"
    "$@"
  else
    printf '+ '
    printf '%q ' "$@"
    printf '
'
  fi
}

while IFS='|' read -r repo_name paths; do
  [[ -z "$repo_name" ]] && continue
  target_dir="$OUTPUT_DIR/$repo_name"
  echo "=== $repo_name ==="
  run_or_print git clone --no-local "$REPO_URL" "$target_dir"

  filter_args=(git -C "$target_dir" filter-repo --force)
  for path in $paths; do
    filter_args+=(--path "$path")
  done
  run_or_print "${filter_args[@]}"
  echo

done <<< "$SPECS"

echo "=== doco_manifests ==="
run_or_print git clone --no-local "$REPO_URL" "$OUTPUT_DIR/doco_manifests"
manifest_args=(git -C "$OUTPUT_DIR/doco_manifests" filter-repo --force)
for path in ${MANIFESTS_SPEC#*|}; do
  manifest_args+=(--path "$path")
done
run_or_print "${manifest_args[@]}"

echo
echo "Next steps after execution:"
echo "1. Rename each repo description / README to match its new ownership."
echo "2. Recreate package remotes and protect branches."
echo "3. Update manifests/*.repos to point at the new repo URLs and tags."
echo "4. Freeze this monorepo into a migration shell once all downstream workspaces consume the manifests repo."
