#!/usr/bin/env bash
# Open RViz with the simulation's MoveIt profile without starting a second
# move_group instance.
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/workspaces.sh"

set +u
source /opt/ros/humble/setup.bash
source "$PIPER_TABLETOP_WS/install/setup.bash"
set -u

SHARE="$(ros2 pkg prefix piper_tabletop_sim)/share/piper_tabletop_sim"
exec rviz2 -d "$SHARE/config/moveit.rviz"
