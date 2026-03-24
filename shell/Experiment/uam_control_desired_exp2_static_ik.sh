#!/bin/bash

# 实验二静态逆解测试：
# 1. 不启动 UAV 侧圆弧轨迹；
# 2. 只启动机械臂执行链和静态 IK 测试节点；
# 3. 用于地面验证“arm_base/arm_target 动捕输入 + 实验二 IK + 零偏补偿”是否稳定。
arm_workspace_environment_var=~/p600_arm_control/devel/setup.bash

gnome-terminal --window -e "bash -c 'source $arm_workspace_environment_var;roslaunch arm_control desired_uam_fly_exp2_static_ik.launch; exec bash'" 