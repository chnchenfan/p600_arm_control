#!/bin/bash

# 任何一步失败就直接退出，避免 ros 环境没配好却继续录包，
# 最后得到一个看似存在、实际上内容不完整的 bag。
set -e

# 这份脚本的目标很单一：
# 1. 为“论文实验一”单独录一份干净的 rosbag
# 2. 只保留成图需要的话题，避免 -a 录包时无关数据太多
# 3. 在同目录额外保存一份元数据，便于后续回溯这次实验到底跑的是哪个分支
#
# 兼容考虑：
# 当前飞机电脑是 Ubuntu 18.04，对应 ROS Melodic，
# 所以默认优先使用 melodic；如果以后在别的机器上运行，
# 再回退到 noetic。
ROS_SETUP="/opt/ros/melodic/setup.bash"
if [ ! -f "$ROS_SETUP" ] && [ -f "/opt/ros/noetic/setup.bash" ]; then
  ROS_SETUP="/opt/ros/noetic/setup.bash"
fi

# 先 source 系统级 ROS 环境，再叠加两个工作空间环境。
# 顺序保持和日常手工启动类似，避免 rospack / rosbag 找不到自定义消息。
if [ -f "$ROS_SETUP" ]; then
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
fi

# 无人机工作空间，里面有 mavros / uav 相关节点与消息。
if [ -f "$HOME/UAV_project/devel/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "$HOME/UAV_project/devel/setup.bash"
fi

# 机械臂工作空间，里面有 uam_message 和机械臂控制相关话题。
if [ -f "$HOME/p600_arm_control/devel/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "$HOME/p600_arm_control/devel/setup.bash"
fi

# 通过第一个位置参数记录这次实验模式，方便之后区分：
# single_joint / dual_joint / custom 等。
# 如果调用时没传，就先记为 unknown，不阻塞录包。
EXPERIMENT_MODE="${1:-unknown}"

# 用时间戳给每次实验单独建目录，避免不同轮次结果互相覆盖。
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_DIR="$HOME/p600_arm_control/data/experiment1/$TIMESTAMP"
BAG_PATH="$OUTPUT_DIR/experiment1_px4.bag"
META_PATH="$OUTPUT_DIR/metadata.txt"

# 先创建输出目录，后续 bag 和元数据都写到这里。
mkdir -p "$OUTPUT_DIR"

ARM_BRANCH="unknown"
UAV_BRANCH="unknown"

# 如果当前目录确实是 git 仓库，就把当前分支名记录下来。
# 这对后面论文出图、复现实验和排查版本差异很重要。
if git -C "$HOME/p600_arm_control" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  ARM_BRANCH="$(git -C "$HOME/p600_arm_control" branch --show-current 2>/dev/null || echo unknown)"
fi

if git -C "$HOME/UAV_project" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  UAV_BRANCH="$(git -C "$HOME/UAV_project" branch --show-current 2>/dev/null || echo unknown)"
fi

# 元数据文件故意保持“key=value”这种简单格式：
# 1. 人眼直接看很方便
# 2. shell / python 后续都很好解析
# 3. 不额外引入 json/yaml 依赖
cat > "$META_PATH" <<EOF
timestamp=$TIMESTAMP
experiment_mode=$EXPERIMENT_MODE
arm_branch=$ARM_BRANCH
uav_branch=$UAV_BRANCH
bag_path=$BAG_PATH
EOF

echo "Experiment 1 record output: $OUTPUT_DIR"
echo "Metadata saved to: $META_PATH"
echo "Press Ctrl-C to stop rosbag recording."

# 下面这组话题是“论文实验一成图最小集合”：
# /vrpn_client_node/Tracker0/pose
#   动捕测得的无人机实际位姿，后续作为基座真实值。
# /mavros/local_position/pose
#   PX4/MAVROS 内部估计位姿，可用于和动捕结果交叉核对。
# /mavros/setpoint_raw/local
#   飞控实际接收到的局部位置期望，用来和真实基座轨迹做跟踪对比。
# /wjl/guidefly/pose_d
#   上层实验节点发布的无人机期望，能帮助判断期望写入端是否正确。
# /wjl/arm/guidefly/angle_d
#   机械臂关节期望角，后续会通过正运动学还原末端参考轨迹。
# /wjl/arm/real/angle_r
#   机械臂关节实测角，后续通过正运动学得到末端实际轨迹。
# /wjl/arm/real/angle_error
#   机械臂关节误差，便于单独做关节空间诊断图。
#
# 这里只录论文图所需话题，不用 rosbag record -a，
# 是为了减小 bag 体积并降低后处理复杂度。
rosbag record -O "$BAG_PATH" \
  /vrpn_client_node/Tracker0/pose \
  /mavros/local_position/pose \
  /mavros/setpoint_raw/local \
  /wjl/guidefly/pose_d \
  /wjl/arm/guidefly/angle_d \
  /wjl/arm/real/angle_r \
  /wjl/arm/real/angle_error
