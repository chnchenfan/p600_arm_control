#!/bin/bash

set -e

# 静态标定专用录包脚本。
#
# 这个脚本不负责控制机械臂，它只负责把“后处理拟合真正需要的最小信息集合”录下来。
# 这里最关键的不是录所有话题，而是录清楚下面这条信息流：
# 1. /vrpn_client_node/arm_base/pose     : 动捕测得的机械臂基座刚体位姿
# 2. /vrpn_client_node/arm_target/pose   : 动捕测得的末端刚体位姿
# 3. /wjl/arm/real/angle_r               : 真机回读的关节角
# 4. /wjl/arm/guidefly/angle_d           : 采样节点发出的目标角
# 5. /wjl/calibration/sample_index       : 哪一段时间属于哪个静态样本的标签
#
# 后续拟合脚本正是依靠 sample_index 对 bag 自动分段，然后在每一段内对 base/target/real_angle 取均值。

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

# metadata.txt 的作用：
# 1. 给这包数据留一份“采样条件说明”；
# 2. 以后回看 bag 时，不需要再去翻代码，就能知道当时的超时、窗口长度和样本顺序。
cat > "$META_PATH" <<META
timestamp=$TIMESTAMP
arm_branch=$ARM_BRANCH
uav_branch=$UAV_BRANCH
bag_path=$BAG_PATH
sample_hold_sec=2.5
settle_confirm_sec=0.5
angle_tolerance_deg=1.0
sample_timeout_sec=15.0
sample_sequence=(-60,0);(-45,0);(-30,0);(-15,0);(0,0);(15,0);(30,0);(45,0);(60,0);(60,15);(30,15);(0,15);(-30,15);(-60,15);(-45,30);(-15,30);(15,30);(45,30);(30,35);(0,35);(-30,35);(-15,40);(0,40);(15,40)
base_pose_topic=/vrpn_client_node/arm_base/pose
ee_pose_topic=/vrpn_client_node/arm_target/pose
arm_real_topic=/wjl/arm/real/angle_r
arm_desired_topic=/wjl/arm/guidefly/angle_d
sample_index_topic=/wjl/calibration/sample_index
META

echo "Static calibration record output: $OUTPUT_DIR"
echo "Metadata saved to: $META_PATH"
echo "Press Ctrl-C to stop rosbag recording."

# 录包话题说明：
# - base/target pose：用于建立动捕几何模型；
# - real/angle_r：用于提供真实关节角；
# - guidefly/angle_d：用于回看当时给了什么命令；
# - sample_index：用于把整包数据自动切成 24 个静态样本窗口。
rosbag record -O "$BAG_PATH"   /vrpn_client_node/arm_base/pose   /vrpn_client_node/arm_target/pose   /wjl/arm/real/angle_r   /wjl/arm/guidefly/angle_d   /wjl/calibration/sample_index
