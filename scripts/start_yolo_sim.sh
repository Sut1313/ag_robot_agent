#!/usr/bin/env bash
# Start the piper_with_yolo_graspnet Gazebo Harmonic simulation (headless).
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/workspaces.sh"

PROJ="$PIPER_YOLO_SIM_PROJECT"

set +u
source /opt/ros/humble/setup.bash
source_gz_control_overlay
source "$PROJ/install/setup.bash"
source "$PROJ/gazebo_piper_pick_place/install/setup.bash"
set -u

if [[ "${YOLO_LIVE_PERCEPTION:-0}" == "1" ]]; then
  exec ros2 launch piper_lightweight_sim lightweight_sim.launch.py \
    gazebo_gui:=false camera_pitch_offset:=1.047197551 \
    start_yolo:=true state_driven_perception:=true
fi

exec ros2 launch piper_lightweight_sim lightweight_sim.launch.py \
  gazebo_gui:=false camera_pitch_offset:=0.0 \
  start_yolo:=false state_driven_perception:=false
