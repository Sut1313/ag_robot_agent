#!/usr/bin/env bash
# Run the piper_with_yolo_graspnet pick pipeline.
# lightweight_pick.launch.py starts MoveIt AND the one-shot executor with the
# robot_description parameters the executor needs; the simulation must already
# be running (start_yolo_sim.sh).
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/workspaces.sh"

PROJ="$PIPER_YOLO_SIM_PROJECT"

set +u
source /opt/ros/humble/setup.bash
source_gz_control_overlay
source "$PROJ/install/setup.bash"
source "$PROJ/gazebo_piper_pick_place/install/setup.bash"
set -u

: "${SHARED_DIR:?SHARED_DIR is required}"

ARGS=(
  "shared_dir:=$SHARED_DIR"
  "dry_run:=${DRY_RUN:-true}"
  "max_candidates:=${MAX_CANDIDATES:-10}"
  "camera_pitch_offset:=${YOLO_CAMERA_PITCH:-0.0}"
  "start_rviz:=false"
)
if [ -n "${TARGET:-}" ]; then
  ARGS+=("target:=$TARGET")
fi

exec ros2 launch piper_lightweight_sim lightweight_pick.launch.py "${ARGS[@]}"
