# !/bin/bash
#说明,注意修改路径
workplace_environment_var=~/UAV_project/devel/setup.bash
gnome-terminal --window -e "bash -c 'source $workplace_environment_var;roslaunch uav uav_drvier_guidefly.launch; exec bash'" \
--tab -e "bash -c 'sleep 1;cd ~/p600_arm_control/src/arm_control/shell/physical;bash serial_start.sh; exec bash'" \
# 定位说明
# w0:gps定位,w1:gazebo真值定位,w2:tof+T265定位,w3:动捕定位