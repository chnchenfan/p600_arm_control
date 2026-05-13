#说明：通过代码给定无人机机械臂的期望数据
workplace_environment_var=~/UAV_project/devel/setup.bash
arm_workspace_environment_var=~/p600_arm_control/devel/setup.bash
gnome-terminal --window -e "bash -c 'source $workplace_environment_var;roslaunch uav desired_uav_fly.launch; exec bash'" \
--tab -e "bash -c 'sleep 1;cd ~/p600_arm_control/src/arm_control/shell;bash desired_uam_fly.sh; exec bash'" \
--tab -e "bash -c 'sleep 1;source ~/p600_arm_control/devel/setup.bash;rosrun arm_control serial_; exec bash'" \
--tab -e "bash -c 'sleep 3;source $arm_workspace_environment_var;roslaunch arm_control arm_eso_px4_bridge.launch; exec bash'"
# 定位说明
# w0:gps定位,w1:gazebo真值定位,w2:tof+T265定位,w3:动捕定位
