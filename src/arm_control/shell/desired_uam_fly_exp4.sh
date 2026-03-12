# !/bin/bash

workplace_environment_var=~/p600_arm_control/devel/setup.bash
gnome-terminal --window -e "bash -c 'sleep 1;source $workplace_environment_var;roslaunch arm_control desired_uam_fly_exp4.launch; exec bash'" \
