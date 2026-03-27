#!/bin/bash

# 实验二自动验证套件：
# 1. 先自动执行 5 组静态 IK 复测；
# 2. 静态组都通过后，再执行虚拟基座扰动测试；
# 3. 虚拟扰动只验证补偿逻辑稳定性，不能替代真实小扰动复验。
arm_workspace_environment_var=~/p600_arm_control/devel/setup.bash

gnome-terminal --window -e "bash -c 'source $arm_workspace_environment_var;roslaunch arm_control desired_uam_fly_exp2_validation.launch; exec bash'"
