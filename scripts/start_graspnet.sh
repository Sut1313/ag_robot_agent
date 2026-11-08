#!/usr/bin/env bash
# Start GraspNet inference (Docker). Real mode only for now: the container
# mounts the workspace's shared_camera_data directory.
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/workspaces.sh"

WS="$PIPER_REAL_WS"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

cd "$WS"
exec ./run_graspnet_yolo.sh
