# !/bin/bash
#说明,雷达建图启动提供位置信息，一个遥控器控制飞机，xbox遥控器控制机械臂
workplace_environment_var=~/p600_arm_control/devel/setup.bash
gnome-terminal --window -e "bash -c 'roslaunch mavros px4.launch; exec bash'" \
--tab -e "bash -c 'sleep 5;source ~/fast_lio/devel/setup.bash;roslaunch fast_lio mapping_mid360.launch; exec bash'" \
--tab -e "bash -c 'sleep 7;source ~/fast_lio/devel/setup.bash;roslaunch livox_ros_driver2 msg_MID360.launch; exec bash'" \
--tab -e "bash -c 'sleep 7;source $workplace_environment_var;roslaunch xbox_control xbox_control_start.launch; exec bash'" \
--tab -e "bash -c 'sleep 7;rostopic echo /mavros/local_position/pose; exec bash'" \



