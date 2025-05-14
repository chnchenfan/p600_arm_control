# !/bin/bash
#说明
#这个脚本文件是在功能包的launch文件夹中
#适用结构：工作区/src/功能包/launch/*.sh
cd ../../..   #工作区路径
workplace_environment_var=$(pwd)/devel/setup.bash #注意看后缀是zsh还是bash
gnome-terminal --window -e "bash -c 'roscore; exec bash'" \
--tab -e "bash -c 'sleep 2;source $workplace_environment_var;rosrun arm_kinematics arm_B_dh.py; exec bash'" \