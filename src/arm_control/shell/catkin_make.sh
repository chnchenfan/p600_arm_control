# !/bin/zsh
cd ../../..  
catkin_make -DCATKIN_WHITELIST_PACKAGES="arm_control"  # 指定要编译的包
