#!/usr/bin/env bash
# Start the YOLO bridge against Gazebo camera topics.
# The simulation publishes RealSense-compatible topic names, so the real
# perception bridge can consume them directly; the physical camera driver must
# NOT be started here.
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/workspaces.sh"

set +u
source /opt/ros/humble/setup.bash
source "$PIPER_TABLETOP_WS/install/setup.bash"
source "$PIPER_REAL_WS/install/setup.bash"
set -u

: "${SHARED_DIR:?SHARED_DIR is required}"

# The simulated scene only contains the 4 cm wooden block.  The 16-class
# tabletop model scores it far below the 0.35 threshold, so use the model
# trained on this scene and use the same class name as tabletop_pick.
export TABLETOP_YOLO_MODEL="${TABLETOP_YOLO_MODEL:-$PIPER_TABLETOP_WS/models/wood_block_yolov8s_seg.onnx}"
export TABLETOP_YOLO_CLASS_NAMES="${TABLETOP_YOLO_CLASS_NAMES:-wood_block}"
export YOLO_ONNX_INTRA_OP_THREADS="${YOLO_ONNX_INTRA_OP_THREADS:-4}"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-4}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
TARGET_CLASS="${TARGET_CLASS:-wood_block}"

exec ros2 run real_piper_yolo_graspnet d435i_yolo_graspnet_bridge --ros-args \
  -p use_sim_time:=true \
  -p model_path:="${TABLETOP_YOLO_MODEL}" \
  -p target_class:="${TARGET_CLASS}" \
  -p fail_on_multiple:=false \
  -p use_calibration_intrinsics:=false \
  -p depth_is_meters:=true \
  -p color_topic:=/camera/camera/color/image_raw \
  -p depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  -p camera_info_topic:=/camera/camera/color/camera_info \
  -p confidence_threshold:=0.25 \
  -p shared_dir:="${SHARED_DIR}" \
  -p publish_rate_hz:=2.0 \
  -p require_consumer:=false
