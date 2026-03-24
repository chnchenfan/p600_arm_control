#!/bin/bash

workplace_environment_var=~/p600_arm_control/devel/setup.bash
# Launch the full static-calibration arm chain: params + bridge + serial_ + collector.
gnome-terminal --window -e "bash -c 'source $workplace_environment_var;roslaunch arm_control uam_static_calibration.launch; exec bash'"
