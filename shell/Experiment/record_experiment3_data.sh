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

EXPERIMENT_MODE="${1:-exp3_square_motion}"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_DIR="$HOME/p600_arm_control/data/experiment3/$TIMESTAMP"
BAG_PATH="$OUTPUT_DIR/experiment3_px4.bag"
META_PATH="$OUTPUT_DIR/metadata.txt"
MONITOR_PATH="$OUTPUT_DIR/experiment3_monitor.csv"
MONITOR_SCRIPT="$HOME/p600_arm_control/shell/Experiment/record_experiment3_monitor.py"
MONITOR_TMP_DIR="$OUTPUT_DIR/monitor_tmp"
DESIRED_TMP="$MONITOR_TMP_DIR/pose_d.csv"
VRPN_TMP="$MONITOR_TMP_DIR/vrpn_pose.csv"
MAVROS_TMP="$MONITOR_TMP_DIR/mavros_pose.csv"
STATE_TMP="$MONITOR_TMP_DIR/mavros_state.csv"

mkdir -p "$OUTPUT_DIR"
mkdir -p "$MONITOR_TMP_DIR"

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
monitor_csv=$MONITOR_PATH
EOF

echo "Experiment 3 record output: $OUTPUT_DIR"
echo "Metadata saved to: $META_PATH"
echo "Readable monitor CSV: $MONITOR_PATH"
echo "Press Ctrl-C to stop rosbag recording."

PYTHON_BIN="python3"
if ! command -v "$PYTHON_BIN" >/dev/null 2>&1 && command -v python >/dev/null 2>&1; then
  PYTHON_BIN="python"
fi

MONITOR_PIDS=""
cleanup() {
  for pid in $MONITOR_PIDS; do
    kill "$pid" >/dev/null 2>&1 || true
    wait "$pid" >/dev/null 2>&1 || true
  done

  if [ -f "$MONITOR_SCRIPT" ] && command -v "$PYTHON_BIN" >/dev/null 2>&1; then
    "$PYTHON_BIN" "$MONITOR_SCRIPT" \
      --desired-csv "$DESIRED_TMP" \
      --vrpn-csv "$VRPN_TMP" \
      --mavros-csv "$MAVROS_TMP" \
      --state-csv "$STATE_TMP" \
      --output "$MONITOR_PATH" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

if [ -f "$MONITOR_SCRIPT" ] && command -v "$PYTHON_BIN" >/dev/null 2>&1; then
  rostopic echo -p /wjl/guidefly/pose_d > "$DESIRED_TMP" &
  MONITOR_PIDS="$MONITOR_PIDS $!"
  rostopic echo -p /vrpn_client_node/Tracker0/pose > "$VRPN_TMP" &
  MONITOR_PIDS="$MONITOR_PIDS $!"
  rostopic echo -p /mavros/local_position/pose > "$MAVROS_TMP" &
  MONITOR_PIDS="$MONITOR_PIDS $!"
  rostopic echo -p /mavros/state > "$STATE_TMP" &
  MONITOR_PIDS="$MONITOR_PIDS $!"
  echo "Readable monitor CSV temp capture started."
else
  echo "Readable monitor logger not started: missing $MONITOR_SCRIPT or python interpreter"
fi

rosbag record -O "$BAG_PATH" \
  /vrpn_client_node/Tracker0/pose \
  /mavros/local_position/pose \
  /mavros/vision_pose/pose \
  /mavros/state \
  /mavros/setpoint_raw/local \
  /wjl/guidefly/pose_d \
  /wjl/arm/guidefly/angle_d \
  /wjl/arm/real/angle_r \
  /wjl/arm/real/angle_error
