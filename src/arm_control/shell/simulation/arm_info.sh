# !/bin/bash

workplace_environment_var=~/p600_arm_control/devel/setup.bash #注意看后缀是zsh还是bash
gnome-terminal --window -e "bash -c 'source $workplace_environment_var;roslaunch arm_control arm_info.launch; exec bash'" \
# --tab -e "bash -c 'sleep 4;source $workplace_environment_var;rosrun rqt_gui rqt_gui -s rqt_reconfigure; exec bash'" \