# !/bin/bash

cd ../../..   #工作区路径
workplace_environment_var=$(pwd)/devel/setup.bash #注意看后缀是zsh还是bash
gnome-terminal --window -e "bash -c 'source $workplace_environment_var;roslaunch xbox_control xbox_control_start.launch; exec bash'" \
