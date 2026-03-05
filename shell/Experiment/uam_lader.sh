#!/bin/bash
# 这是动捕定位的启动脚本,启动px4文件和定位估计
sudo chmod 777 /dev/ttyTHS0
source ~/.bashrc

## 定位说明
# 这里定位默认使用动捕定位
workplace_environment_var=~/UAV_project/devel/setup.bash
gnome-terminal --window -e "bash -c 'source $workplace_environment_var;roslaunch uav uav_yaml.launch estimator_flag:=w3; exec bash'" \
--tab -e "bash -c 'sleep 3;source $workplace_environment_var;roslaunch uav real_px4_startup.launch ns_flag:=g0; exec bash'" \
--tab -e "bash -c 'sleep 5;source ~/fast_lio/devel/setup.bash;roslaunch fast_lio mapping_mid360.launch; exec bash'" \
--tab -e "bash -c 'sleep 7;source ~/fast_lio/devel/setup.bash;roslaunch livox_ros_driver2 msg_MID360.launch; exec bash'" \
--tab -e "bash -c 'sleep 4; rostopic echo /mavros/local_position/pose; exec bash'" \