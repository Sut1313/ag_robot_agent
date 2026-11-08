#!/usr/bin/env bash
# Start MoveIt for piper_with_yolo_graspnet (Gazebo classic).
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/workspaces.sh"

PROJ="$PIPER_YOLO_SIM_PROJECT"

set +u
source /opt/ros/humble/setup.bash
source_gz_control_overlay
source "$PROJ/install/setup.bash"
source "$PROJ/gazebo_piper_pick_place/install/setup.bash"
set -u

exec ros2 launch piper_lightweight_sim lightweight_moveit.launch.py start_rviz:=false
