#!/usr/bin/env bash
# Source before loading ROS overlays.
AGENT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
: "${PIPER_REAL_WS:=$AGENT_ROOT/../real_piper_yolo_graspnet}"
: "${PIPER_TABLETOP_WS:=$AGENT_ROOT/../piper_tabletop_sim_ws}"
: "${PIPER_YOLO_SIM_PROJECT:=$AGENT_ROOT/../piper_with_yolo_graspnet}"
: "${PIPER_GZ_CONTROL_WS:=$HOME/gz_ros2_control_ws}"

source_gz_control_overlay() {
  if [ -r "$PIPER_GZ_CONTROL_WS/install/setup.bash" ]; then
    source "$PIPER_GZ_CONTROL_WS/install/setup.bash"
  fi
}
