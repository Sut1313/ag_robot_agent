#!/usr/bin/env bash
# Start MoveIt move_group for the Gazebo simulation.
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/workspaces.sh"

set +u
source /opt/ros/humble/setup.bash
source "$PIPER_TABLETOP_WS/install/setup.bash"
set -u

exec ros2 launch piper_tabletop_sim move_group.launch.py \
  start_rviz:="${START_RVIZ:-false}"
