#!/usr/bin/env bash
# Immediate stop, usable from another terminal without the agent.
#   ./estop.sh sim    deactivate arm/gripper controllers
#   ./estop.sh real   the above + disable the Piper arm
set -uo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/workspaces.sh"

MODE="${1:-sim}"

set +u
source /opt/ros/humble/setup.bash
set -u

echo "[estop] deactivating arm_controller / gripper_controller"
ros2 control switch_controllers \
  --deactivate arm_controller gripper_controller --strict \
  || echo "[estop] controller deactivation failed (already inactive?)"

if [ "$MODE" = "real" ]; then
  set +u
  source "$PIPER_REAL_WS/install/setup.bash"
  set -u
  echo "[estop] disabling Piper arm"
  ros2 service call /enable_srv piper_msgs/srv/Enable \
    "{enable_request: false}" \
    || echo "[estop] arm disable failed"
fi

echo "[estop] done"
