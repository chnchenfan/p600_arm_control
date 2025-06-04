# !/bin/bash
#说明
workplace_environment_var=~/p600_arm_control/devel/setup.bash #注意看后缀是zsh还是bash
gnome-terminal --window -e "bash -c 'source $workplace_environment_var;roslaunch arm_control arm_physical_control.launch; exec bash'" \
--tab -e "bash -c 'sleep 2;source $workplace_environment_var;rosrun arm_control motors_simulation; exec bash'" \