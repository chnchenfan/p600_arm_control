#!/bin/bash

set -e

usage() {
  cat <<'EOF'
用法:
  bash shell/Experiment/record_experiment_data.sh exp1 [实验备注]
  bash shell/Experiment/record_experiment_data.sh exp2 [实验备注]
  bash shell/Experiment/record_experiment_data.sh exp3 [实验备注]
  bash shell/Experiment/record_experiment_data.sh exp4 [实验备注]

说明:
  该脚本统一记录 exp1~exp4 的诊断 rosbag。缺失话题只在终端中文警告，不中止录包。
EOF
}

EXPERIMENT="${1:-}"
case "$EXPERIMENT" in
  exp1|exp2|exp3|exp4)
    ;;
  *)
    usage
    exit 1
    ;;
esac

EXPERIMENT_MODE="${2:-$EXPERIMENT}"

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
OUTPUT_DIR="$HOME/p600_arm_control/data/$EXPERIMENT/$TIMESTAMP"
BAG_PATH="$OUTPUT_DIR/${EXPERIMENT}_px4.bag"
META_PATH="$OUTPUT_DIR/metadata.txt"

mkdir -p "$OUTPUT_DIR"

ARM_BRANCH="unknown"
ARM_COMMIT="unknown"
UAV_BRANCH="unknown"
UAV_COMMIT="unknown"

if git -C "$HOME/p600_arm_control" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  ARM_BRANCH="$(git -C "$HOME/p600_arm_control" branch --show-current 2>/dev/null || echo unknown)"
  ARM_COMMIT="$(git -C "$HOME/p600_arm_control" rev-parse --short HEAD 2>/dev/null || echo unknown)"
fi

if git -C "$HOME/UAV_project" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  UAV_BRANCH="$(git -C "$HOME/UAV_project" branch --show-current 2>/dev/null || echo unknown)"
  UAV_COMMIT="$(git -C "$HOME/UAV_project" rev-parse --short HEAD 2>/dev/null || echo unknown)"
fi

AVAILABLE_TOPICS="$(rostopic list 2>/dev/null || true)"
if [ -z "$AVAILABLE_TOPICS" ]; then
  echo "警告：当前无法从 rostopic list 获取话题列表。请确认 roscore/MAVROS/动捕节点已经启动；脚本仍会继续启动 rosbag。"
fi

TOPICS_CONFIGURED=""
TOPICS_AVAILABLE=""
TOPICS_MISSING=""
TOPICS_RECORD_ARGS=""

append_csv() {
  key_value="$1"
  item="$2"
  if [ -z "$key_value" ]; then
    printf '%s' "$item"
  else
    printf '%s,%s' "$key_value" "$item"
  fi
}

echo "开始检查录包话题："
while IFS='|' read -r category topic description; do
  [ -n "$topic" ] || continue
  TOPICS_CONFIGURED="$(append_csv "$TOPICS_CONFIGURED" "$topic")"
  TOPICS_RECORD_ARGS="$TOPICS_RECORD_ARGS $topic"
  if printf '%s\n' "$AVAILABLE_TOPICS" | grep -Fxq "$topic"; then
    echo "已检测到 [$category] $topic：$description"
    TOPICS_AVAILABLE="$(append_csv "$TOPICS_AVAILABLE" "$topic")"
  else
    echo "警告：未检测到 [$category] $topic：$description，本次 bag 将无法分析该项；若飞行中该话题后续出现，rosbag 仍会开始记录。"
    TOPICS_MISSING="$(append_csv "$TOPICS_MISSING" "$topic")"
  fi
done <<'TOPICS'
上层与飞控setpoint|/wjl/guidefly/pose_d|上层无人机位置/偏航期望
上层与飞控setpoint|/mavros/setpoint_raw/local|MAVROS实际送入PX4的local setpoint
上层与飞控setpoint|/mavros/setpoint_raw/attitude|MAVROS姿态/推力期望
上层与飞控setpoint|/mavros/setpoint_raw/target_attitude|MAVROS raw attitude target，如果该MAVROS版本发布
实际位置/速度|/mavros/local_position/pose|PX4/MAVROS local实际位置
实际位置/速度|/mavros/local_position/velocity_local|PX4/MAVROS local实际速度
实际位置/速度|/mavros/vision_pose/pose|视觉/动捕输入给MAVROS的位姿
实际姿态/角速度/IMU|/mavros/imu/data|滤波后的IMU姿态和角速度
实际姿态/角速度/IMU|/mavros/imu/data_raw|原始IMU加速度和角速度
电机/推力/ESC|/mavros/actuator_control|MAVROS actuator control输出，如果该链路存在
电机/推力/ESC|/mavros/esc_status|ESC转速/电流/温度状态，如果该链路存在
飞控状态|/mavros/state|连接、模式、armed状态
飞控状态|/mavros/extended_state|landed等扩展状态
飞控状态|/mavros/rc/in|遥控器输入
飞控状态|/diagnostics|ROS/MAVROS诊断信息
电池|/mavros/battery|电压、电流、电量状态
机械臂期望/实际|/wjl/arm/guidefly/angle_d|机械臂上层期望角
机械臂期望/实际|/wjl/arm/real/angle_d|机械臂限幅/执行层目标角
机械臂期望/实际|/wjl/arm/real/angle_r|机械臂实际反馈角
机械臂期望/实际|/wjl/arm/real/angle_error|机械臂关节误差
机械臂期望/实际|/wjl/arm/guidefly/online_offset|实验二等在线补偿量；不存在则警告
动捕|/vrpn_client_node/arm_base/pose|动捕机体/基座位姿
坐标变换|/tf|动态坐标变换
坐标变换|/tf_static|静态坐标变换
TOPICS

{
  echo "timestamp=$TIMESTAMP"
  echo "experiment=$EXPERIMENT"
  echo "experiment_mode=$EXPERIMENT_MODE"
  echo "arm_branch=$ARM_BRANCH"
  echo "arm_commit=$ARM_COMMIT"
  echo "uav_branch=$UAV_BRANCH"
  echo "uav_commit=$UAV_COMMIT"
  echo "bag_path=$BAG_PATH"
  echo "topics_configured=${TOPICS_CONFIGURED:-none}"
  echo "topics_available=${TOPICS_AVAILABLE:-none}"
  echo "topics_missing=${TOPICS_MISSING:-none}"
} > "$META_PATH"

echo "实验录包输出目录: $OUTPUT_DIR"
echo "元数据文件: $META_PATH"
echo "bag文件: $BAG_PATH"
echo "按 Ctrl-C 停止 rosbag record。"

rosbag record -O "$BAG_PATH" $TOPICS_RECORD_ARGS
