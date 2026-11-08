#!/usr/bin/env bash
set -eo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/workspaces.sh"

source /opt/ros/humble/setup.bash
source_gz_control_overlay
source "$PIPER_TABLETOP_WS/install/setup.bash"
source "$PIPER_REAL_WS/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${TABLETOP_ROS_DOMAIN_ID:-67}"
export GZ_PARTITION="${TABLETOP_GZ_PARTITION:-tabletop67}"
TABLETOP_SHARED_DIR="${TABLETOP_SHARED_DIR:-$AGENT_ROOT/runtime_data/tabletop_67}"
mkdir -p "${TABLETOP_SHARED_DIR}"

case "${1:?module required}" in
  sim)
    exec ros2 launch piper_tabletop_sim sim.launch.py gazebo_gui:=false
    ;;
  moveit)
    exec ros2 launch piper_tabletop_sim move_group.launch.py start_rviz:=false
    ;;
  feedback)
    exec python3 "$PIPER_TABLETOP_WS/sim_support/joint_state_feedback_relay.py"
    ;;
  yolo)
    export TABLETOP_YOLO_CLASS_NAMES=wood_block
    export YOLO_ONNX_INTRA_OP_THREADS="${YOLO_ONNX_INTRA_OP_THREADS:-4}"
    export OMP_NUM_THREADS="${OMP_NUM_THREADS:-4}"
    export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
    exec ros2 run real_piper_yolo_graspnet d435i_yolo_graspnet_bridge --ros-args \
      -p model_path:="$PIPER_TABLETOP_WS/models/wood_block_yolov8s_seg.onnx" \
      -p target_class:=wood_block \
      -p use_calibration_intrinsics:=false \
      -p depth_is_meters:=true \
      -p shared_dir:="${TABLETOP_SHARED_DIR}" \
      -p color_topic:=/camera/camera/color/image_raw \
      -p depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
      -p camera_info_topic:=/camera/camera/color/camera_info \
      -p publish_rate_hz:=2.0 \
      -p confidence_threshold:=0.25
    ;;
  graspnet)
    export SHARED_DIR="${TABLETOP_SHARED_DIR}"
    export GRASPNET_CONTAINER_NAME="agrobot_tabletop_${ROS_DOMAIN_ID}"
    export GRASPNET_CPU_LIMIT="${GRASPNET_CPU_LIMIT:-4}"
    export GRASPNET_OMP_THREADS="${GRASPNET_OMP_THREADS:-4}"
    exec "$PIPER_REAL_WS/run_graspnet_yolo.sh" headless
    ;;
  executor)
    shift
    export GRASP_GRIPPER_SETTLE_S=5.0
    exec ros2 launch real_piper_perception grasp_execute.launch.py "$@"
    ;;
  scene)
    exec python3 "$PIPER_TABLETOP_WS/sim_support/add_table_scene.py"
    ;;
  ros2)
    shift
    exec ros2 "$@"
    ;;
  gz)
    shift
    exec gz "$@"
    ;;
  *) echo "unknown tabletop module: $1" >&2; exit 2 ;;
esac
