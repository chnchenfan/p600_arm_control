# !/bin/bash
# 无人机机械臂gazebo仿真启动调节机械臂PID控制
workplace_environment_var=~/UAV_project/devel/setup.bash
gnome-terminal --window -e "bash -c 'cd ~/p600_arm_control/src/arm_control/shell/simulation;zsh arm_pid_set.sh; exec bash'" \
--tab -e "bash -c 'sleep 2;source $workplace_environment_var;roslaunch uav uav_yaml.launch estimator_flag:=w1; exec bash'" \
--tab -e "bash -c 'sleep 3;source $workplace_environment_var;roslaunch uav uav_drvier_guidefly.launch; exec bash'" \
--tab -e "bash -c 'sleep 2.5;
source $workplace_environment_var;
export ROS_PACKAGE_PATH=/home/wjl/PX4-Autopilot:\$ROS_PACKAGE_PATH;
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/sitl_gazebo;
roslaunch uav gazebo_px4_startup.launch ns_flag:=g0 sdf_model_uav_:=uam_v4;exec bash'" \

##  组名说明
# 启动uav中的launch文件前面的申明的所有变量是为了保证环境变量都能正常用
# 需要和config中的group_flag标志对应
# group_flag:0  启动  gazebo_px4_startup.launch ns_flag:=g0  无组名
# group_flag:1  启动  gazebo_px4_startup.launch ns_flag:=g1  有组名

## 定位说明
# gazebo仿真默认使用gazbeo真值定位,见roslaunch uav uav_yaml.launch estimator_flag:=w1
# w0:gps定位,w1:gazebo真值定位,w2:tof+T265定位,w3:动捕定位