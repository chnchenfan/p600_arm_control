#!/bin/bash
cd ~/p600_arm_control 
catkin_make -DCATKIN_WHITELIST_PACKAGES="uam_message"
catkin_make -DCATKIN_WHITELIST_PACKAGES="arm_control"

catkin_make -DCATKIN_WHITELIST_PACKAGES="xbox_control"