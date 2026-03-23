#!/bin/bash
# 动捕 + PX4 启动入口。
# 用法：
#   bash uam_mocap.sh                    # 默认 Tracker0 + 默认 server
#   bash uam_mocap.sh exp2              # 实验二 -> arm_base
#   bash uam_mocap.sh exp2 10.1.1.198   # 实验二 + 显式指定 VRPN server IP
#   bash uam_mocap.sh arm_target 10.1.1.198

cd ~/UAV_project/shell/Experiment || exit 1
bash pos_mocap.sh "$@"
