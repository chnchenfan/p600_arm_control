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

EXPERIMENT_MODE="${1:-exp2_circle_hold}"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_DIR="$HOME/p600_arm_control/data/experiment2/$TIMESTAMP"
BAG_PATH="$OUTPUT_DIR/experiment2_px4.bag"
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

cat > "$META_PATH" <<EOF
timestamp=$TIMESTAMP
experiment_mode=$EXPERIMENT_MODE
arm_branch=$ARM_BRANCH
uav_branch=$UAV_BRANCH
bag_path=$BAG_PATH
EOF

echo "Experiment 2 record output: $OUTPUT_DIR"
echo "Metadata saved to: $META_PATH"
echo "Press Ctrl-C to stop rosbag recording."

rosbag record -O "$BAG_PATH" \
  /vrpn_client_node/Tracker0/pose \
  /mavros/local_position/pose \
  /mavros/setpoint_raw/local \
  /wjl/guidefly/pose_d \
  /wjl/arm/guidefly/angle_d \
  /wjl/arm/real/angle_r \
  /wjl/arm/real/angle_error
