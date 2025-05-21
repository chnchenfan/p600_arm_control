# !/bin/zsh
cd ../../..  
catkin_make -DCATKIN_WHITELIST_PACKAGES="linux_serial"  # 指定要编译的包
