# !/bin/bash

cd ../../..   #工作区路径
workplace_environment_var=$(pwd)/devel/setup.bash #注意看后缀是zsh还是bash
gnome-terminal --window -e "bash -c 'source $workplace_environment_var;roslaunch linux_serial usb.launch; exec bash'" \
--tab -e "bash -c 'sleep 4;source $workplace_environment_var;rosrun rqt_gui rqt_gui -s rqt_reconfigure; exec bash'" \
--tab -e "bash -c 'sleep 4;source $workplace_environment_var;rosrun linux_serial dynamic_angle; exec bash'" \