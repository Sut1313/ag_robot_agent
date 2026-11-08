#!/usr/bin/env bash
# Open MoveIt RViz for the physical Piper arm.
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/workspaces.sh"

WS="$PIPER_REAL_WS"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

exec ros2 launch piper_with_gripper_moveit moveit_rviz.launch.py
