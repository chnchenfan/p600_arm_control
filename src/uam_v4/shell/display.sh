# !/bin/zsh
cd ../../..   #工作区路径
workplace_environment_var=$(pwd)/devel/setup.bash #注意看后缀是zsh还是bash
gnome-terminal --window -e "bash -c 'source $workplace_environment_var;roslaunch uam_v4 display.launch; exec bash'" \
--tab -e "bash -c 'sleep 2;source $workplace_environment_var;roslaunch uam_v4 gazebo.launch; exec bash'" \
