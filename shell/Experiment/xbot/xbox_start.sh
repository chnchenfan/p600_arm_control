# !/bin/bash
# 单纯的xbox遥控器启动
workplace_environment_var=~/p600_arm_control/devel/setup.bash
gnome-terminal --window -e "bash -c 'source $workplace_environment_var;roslaunch xbox_control xbox_control_start.launch; exec bash'" \
