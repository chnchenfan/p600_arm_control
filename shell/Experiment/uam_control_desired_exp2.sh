# 说明：通过代码给定无人机机械臂的实验二期望数据
# 机械臂侧的 desired_uam_fly_exp2.launch 现在会在正式实验开始前，先自动执行一次 arm1=arm2=0° 的上电归零。
uav_workspace_environment_var=~/UAV_project/devel/setup.bash
arm_workspace_environment_var=~/p600_arm_control/devel/setup.bash
gnome-terminal --window -e "bash -c 'source $uav_workspace_environment_var;roslaunch uav desired_uav_fly.launch; exec bash'" \
--tab -e "bash -c 'sleep 1;cd ~/p600_arm_control/src/arm_control/shell;bash desired_uam_fly_exp2.sh; exec bash'" \
--tab -e "bash -c 'sleep 1;source $arm_workspace_environment_var;rosrun arm_control serial_; exec bash'" \
--tab -e "bash -c 'sleep 3;source $arm_workspace_environment_var;roslaunch arm_control arm_eso_px4_bridge.launch; exec bash'"
