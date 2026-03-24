#!/bin/bash

set -e

ROS_SETUP="/opt/ros/melodic/setup.bash"
if [ ! -f "$ROS_SETUP" ] && [ -f "/opt/ros/noetic/setup.bash" ]; then
  ROS_SETUP="/opt/ros/noetic/setup.bash"
fi

if [ -f "$ROS_SETUP" ]; then
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
fi

if [ -f "$HOME/UAV_project/devel/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "$HOME/UAV_project/devel/setup.bash"
fi

if [ -f "$HOME/p600_arm_control/devel/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "$HOME/p600_arm_control/devel/setup.bash"
fi

TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_DIR="$HOME/p600_arm_control/data/calibration/static/$TIMESTAMP"
BAG_PATH="$OUTPUT_DIR/calibration_static.bag"
META_PATH="$OUTPUT_DIR/metadata.txt"

mkdir -p "$OUTPUT_DIR"

ARM_BRANCH="unknown"
UAV_BRANCH="unknown"

if git -C "$HOME/p600_arm_control" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  ARM_BRANCH="$(git -C "$HOME/p600_arm_control" branch --show-current 2>/dev/null || echo unknown)"
fi

if git -C "$HOME/UAV_project" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  UAV_BRANCH="$(git -C "$HOME/UAV_project" branch --show-current 2>/dev/null || echo unknown)"
fi

cat > "$META_PATH" <<META
timestamp=$TIMESTAMP
arm_branch=$ARM_BRANCH
uav_branch=$UAV_BRANCH
bag_path=$BAG_PATH
sample_hold_sec=2.5
settle_confirm_sec=0.5
angle_tolerance_deg=1.0
sample_timeout_sec=15.0
sample_sequence=(-60,0);(-45,15);(-30,30);(-30,0);(-15,15);(0,30);(0,0);(15,15);(30,30);(30,0);(45,15);(60,0)
base_pose_topic=/vrpn_client_node/arm_base/pose
ee_pose_topic=/vrpn_client_node/arm_target/pose
arm_real_topic=/wjl/arm/real/angle_r
arm_desired_topic=/wjl/arm/guidefly/angle_d
sample_index_topic=/wjl/calibration/sample_index
META

echo "Static calibration record output: $OUTPUT_DIR"
echo "Metadata saved to: $META_PATH"
echo "Press Ctrl-C to stop rosbag recording."

rosbag record -O "$BAG_PATH" \
  /vrpn_client_node/arm_base/pose \
  /vrpn_client_node/arm_target/pose \
  /wjl/arm/real/angle_r \
  /wjl/arm/guidefly/angle_d \
  /wjl/calibration/sample_index
