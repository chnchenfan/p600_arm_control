#!/bin/bash
# 动捕 + PX4 启动入口。
# 用法：
#   bash uam_mocap.sh            # 默认 Tracker0
#   bash uam_mocap.sh exp2       # 实验二 -> arm_base
#   bash uam_mocap.sh exp4       # 实验四 -> arm_target
#   bash uam_mocap.sh arm_base   # 直接指定刚体名

cd ~/UAV_project/shell/Experiment || exit 1
bash pos_mocap.sh "$@"
