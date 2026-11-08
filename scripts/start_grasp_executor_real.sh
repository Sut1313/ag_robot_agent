#!/usr/bin/env bash
# Start the grasp executor in REAL mode (scan_only:=false). The arm moves when
# the service is triggered; this script only starts the node.
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
export GRASP_GRIPPER_SETTLE_S="${GRASP_GRIPPER_SETTLE_S:-0.8}"

ARGS=(
  "scan_only:=false"
  "shared_dir:=$SHARED_DIR"
  "handeye_yaml:=$HANDEYE_YAML"
  "velocity_scale:=${VELOCITY_SCALE:-0.02}"
  "open_gripper_lift_height:=${OPEN_GRIPPER_LIFT_HEIGHT:-0.05}"
  "lift_z:=${LIFT_Z:-0.05}"
  "gripper_backend:=${GRIPPER_BACKEND:-sdk}"
  "enable_sim_lift:=${ENABLE_SIM_LIFT:-false}"
  "enable_real_hold_place:=${ENABLE_REAL_HOLD_PLACE:-false}"
  "real_hold_place_motion:=${REAL_HOLD_PLACE_MOTION:-false}"
)
if [ "${ENABLE_REAL_HOLD_PLACE:-false}" = "true" ]; then
  ARGS+=("grasp_z_offset:=${REAL_GRASP_Z_OFFSET_M:-0.0}")
fi
if [ -n "${DRY_RUN_MAX_STAGE:-}" ]; then
  ARGS+=("dry_run_max_stage:=$DRY_RUN_MAX_STAGE")
fi

exec ros2 launch "$AGENT/launch/grasp_executor_real.launch.py" "${ARGS[@]}"
