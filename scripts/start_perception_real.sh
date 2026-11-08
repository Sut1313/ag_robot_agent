#!/usr/bin/env bash
# Start the physical perception stack: RealSense + YOLO bridge + hand-eye TF.
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/workspaces.sh"

WS="$PIPER_REAL_WS"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

export TARGET_CLASS="${TARGET_CLASS:-cup}"
exec "$WS/run_d435i_yolo_bridge.sh"
