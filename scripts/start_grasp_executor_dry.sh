#!/usr/bin/env bash
# Start the grasp executor in dry-run mode (no arm motion).
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/workspaces.sh"

WS="$PIPER_REAL_WS"
AGENT="$AGENT_ROOT"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

: "${SHARED_DIR:?SHARED_DIR is required}"
: "${HANDEYE_YAML:?HANDEYE_YAML is required}"
export GRASP_GRIPPER_SETTLE_S="${GRASP_GRIPPER_SETTLE_S:-5.0}"

ARGS=(
  "scan_only:=true"
  "shared_dir:=$SHARED_DIR"
  "handeye_yaml:=$HANDEYE_YAML"
  "velocity_scale:=${VELOCITY_SCALE:-0.02}"
  "open_gripper_lift_height:=${OPEN_GRIPPER_LIFT_HEIGHT:-0.05}"
  "lift_z:=${LIFT_Z:-0.05}"
  "gripper_backend:=${GRIPPER_BACKEND:-ros}"
  "enable_sim_lift:=${ENABLE_SIM_LIFT:-true}"
)
if [ -n "${DRY_RUN_MAX_STAGE:-}" ]; then
  ARGS+=("dry_run_max_stage:=$DRY_RUN_MAX_STAGE")
fi

exec ros2 launch "$AGENT/launch/grasp_executor_dry.launch.py" "${ARGS[@]}"
