project(Plugin)
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
# Find Gazebo
find_package(gazebo REQUIRED)

string(ASCII 27 Esc)
set(R "${Esc}[0;31m")   #红色
#Esc[0m 
set(E "${Esc}[m" )      #结束颜色设置
set(G "${Esc}[1;32m")   #蓝色高亮

include_directories(
    ${GAZEBO_INCLUDE_DIRS}
    "~/p600_arm_control/devel/include"
)
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(SOME_PATH "~/p600_arm_control/devel/include")
# 展开路径

# 编译可执行文件函数
function(make_build)
    add_library(Uam_pos_pid_v4 SHARED Uam_pos_pid_v4.cc)
    target_link_libraries(Uam_pos_pid_v4 ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})
    add_library(Get_pos_vel_v4 SHARED Get_pos_vel_v4.cc)
    target_link_libraries(Get_pos_vel_v4 ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})
endfunction()

file(TO_CMAKE_PATH ${SOME_PATH} EXPANDED_PATH)#有~路径符号时需要用到他
if(EXISTS ${EXPANDED_PATH})
    message("${G}该路径存在，准备编译...${E}")
    make_build()  #函数
else()
    message("${R}该路径不存在，请编译uam_message功能包!!!${E}")
endif()




