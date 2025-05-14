# !/bin/zsh
cd ../../..  
catkin_make -DCATKIN_WHITELIST_PACKAGES="arm_kinematics"  # 指定要编译的包
