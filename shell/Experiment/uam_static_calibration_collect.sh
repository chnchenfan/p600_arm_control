#!/bin/bash

# 静态标定采样入口。
#
# 这个脚本本身不直接控制机械臂，只负责拉起下面这一整条 ROS 链：
# 1. uam_static_calibration.launch
# 2. launch 内部先加载机械臂参数 arm_info_init.yaml
# 3. launch 起 motors_simulation：把 /wjl/arm/guidefly/angle_d 转成 /wjl/arm/real/angle_d
# 4. launch 起 serial_：把 /wjl/arm/real/angle_d 发给真机步进电机，并回读 /wjl/arm/real/angle_r
# 5. launch 起 uam_static_calibration_collect：先执行一次上电归零到 arm1=arm2=0°，再按预设姿态序列依次发布命令，并根据 /wjl/arm/real/angle_r 判断是否到位
#
# 也就是说，这个脚本只是“总开关”；真正的信息流在 launch 和 collector 节点里完成。

workplace_environment_var=~/p600_arm_control/devel/setup.bash

# 启动完整静态标定链路：参数 + 话题桥 + 串口驱动 + 采样节点。
gnome-terminal --window -e "bash -c 'source $workplace_environment_var;roslaunch arm_control uam_static_calibration.launch; exec bash'"
