# p600_arm_control

P600 UAV arm-side workspace. This repository contains the arm model, arm execution nodes, experiment-level reference generators, recording scripts, and offline plotting tools. It works together with [UAV_project](https://github.com/chnchenfan/P600_uav_control) through shared ROS topics and services to run the four UAV-arm experiments.

GitHub: <https://github.com/chnchenfan/p600_arm_control>  
Companion flight-control repository: <https://github.com/chnchenfan/P600_uav_control>

## Overview

- Recommended environment: Ubuntu 18.04 + ROS Melodic
- Arm control and experiment nodes live in `src/arm_control`
- Custom messages, configs, and visualization presets live in `src/uam_message`
- UAV-arm model assets live in `src/uam_v4`
- Experiment startup, recording, and plotting scripts live in `shell/`

Key directories:

- `src/arm_control/src`: arm execution nodes and experiment 1/2/3/4 reference generators
- `src/arm_control/launch`: simulation and physical launch entrypoints
- `src/arm_control/shell`: arm-side helper launch scripts
- `shell/Experiment`: combined experiment startup and rosbag recording scripts
- `shell/Plot`: offline plotting scripts for experiments 1/2/3/4
- `src/arm_control/README_calibration.md`: staged static calibration workflow for experiment 2 geometry
- Static calibration is split as: aircraft-side collection + host-side offline fitting

## Information Flow

The system runs as: experiment reference generation -> UAV execution + arm execution -> feedback -> logging and offline analysis.

1. Experiment nodes publish unified references
- `uam_desired.cpp`
- `uam_desired_exp2.cpp`
- `uam_desired_exp3.cpp`
- `uam_desired_exp4.cpp`

These nodes expose:

- service `/wjl/start/uav_desired`
- UAV reference `/wjl/guidefly/pose_d`
- arm reference `/wjl/arm/guidefly/angle_d`

2. UAV-side execution
- `desired_uav_fly.launch` in `UAV_project` starts the flight execution chain
- `UAV_project/src/uav/src/uav/Uav_info.cpp` subscribes to `/wjl/guidefly/pose_d`
- UAV position, yaw, and landing commands are converted into PX4/MAVROS-side execution commands

3. Arm-side execution
- Physical arm: `rosrun arm_control serial_`
- Simulation: `motors_simulation`

These nodes consume `/wjl/arm/guidefly/angle_d` and publish:

- `/wjl/arm/real/angle_r`
- `/wjl/arm/real/angle_error`

4. Feedback inputs used by experiment logic

- `/mavros/local_position/pose`: actual UAV base pose
- `/mavros/state`: MAVROS connection and mode state
- `/vrpn_client_node/Tracker0/pose`: mocap truth of the base
- `/wjl/arm/real/angle_r`: measured arm joint angles
- Experiment 4 additionally uses `/vrpn_client_node/ring1/pose` to `/vrpn_client_node/ring4/pose`

5. Logging and post-processing

- `shell/Experiment/record_experiment1_data.sh`
- `shell/Experiment/record_experiment2_data.sh`
- `shell/Experiment/record_experiment3_data.sh`
- `shell/Experiment/record_experiment4_data.sh`

Offline plotting scripts:

- `shell/Plot/plot_experiment1.py`
- `shell/Plot/plot_experiment2.py`
- `shell/Plot/plot_experiment3.py`
- `shell/Plot/plot_experiment4.py`

## The Four Experiments

### Experiment 1: Hover with arm disturbance baseline

Goal:
- Keep the UAV at a fixed hover point
- Drive the arm with periodic two-joint motion
- Observe base tracking and end-effector disturbance response

Main entrypoints:
- Control node: `src/arm_control/src/uam_desired.cpp`
- Combined startup: `shell/Experiment/uam_control_desired.sh`
- Recording: `shell/Experiment/record_experiment1_data.sh`
- Plotting: `shell/Plot/plot_experiment1.py`

### Experiment 2: Circular base motion with end-effector hold compensation

Goal:
- Fly the UAV along a small circle in the world frame
- Use online inverse kinematics to keep the end effector close to a fixed world point

Main entrypoints:
- Control node: `src/arm_control/src/uam_desired_exp2.cpp`
- Arm-side startup: `src/arm_control/shell/desired_uam_fly_exp2.sh`
- Combined startup: `shell/Experiment/uam_control_desired_exp2.sh`
- Recording: `shell/Experiment/record_experiment2_data.sh`
- Plotting: `shell/Plot/plot_experiment2.py`

### Experiment 3: Square base motion with periodic arm motion

Goal:
- Fly the UAV along a square trajectory
- Run periodic two-joint arm motion in parallel
- Add a safety layer based on MAVROS and mocap feedback

Main entrypoints:
- Control node: `src/arm_control/src/uam_desired_exp3.cpp`
- Arm-side startup: `src/arm_control/shell/desired_uam_fly_exp3.sh`
- Combined startup: `shell/Experiment/uam_control_desired_exp3.sh`
- Recording: `shell/Experiment/record_experiment3_data.sh`
- Plotting: `shell/Plot/plot_experiment3.py`

### Experiment 4: Aerial ring passage

Goal:
- Pass the end effector through `ring1 -> ring2 -> ring3 -> ring4` in sequence
- Use mocap-provided ring positions in real time
- Let the UAV base handle large transport motion and the arm handle local alignment
- Keep the gripper closed throughout the mission, without onboard grasping logic

Main entrypoints:
- Control node: `src/arm_control/src/uam_desired_exp4.cpp`
- Arm-side startup: `src/arm_control/shell/desired_uam_fly_exp4.sh`
- Combined startup: `shell/Experiment/uam_control_desired_exp4.sh`
- Recording: `shell/Experiment/record_experiment4_data.sh`
- Plotting: `shell/Plot/plot_experiment4.py`

Additional experiment 4 inputs:

- `/vrpn_client_node/ring1/pose`
- `/vrpn_client_node/ring2/pose`
- `/vrpn_client_node/ring3/pose`
- `/vrpn_client_node/ring4/pose`

## Common Entrypoints

Combined experiment startup scripts:

- Experiment 1: `shell/Experiment/uam_control_desired.sh`
- Experiment 2: `shell/Experiment/uam_control_desired_exp2.sh`
- Experiment 3: `shell/Experiment/uam_control_desired_exp3.sh`
- Experiment 4: `shell/Experiment/uam_control_desired_exp4.sh`

Simulation launch files:

- Experiment 1: `src/arm_control/launch/simulation/desired_uam_fly.launch`
- Experiment 2: `src/arm_control/launch/simulation/desired_uam_fly_exp2.launch`
- Experiment 3: `src/arm_control/launch/simulation/desired_uam_fly_exp3.launch`
- Experiment 4: `src/arm_control/launch/simulation/desired_uam_fly_exp4.launch`

## Notes

- This repository is the arm and experiment-task layer, not the standalone flight-control repository.
- Real experiments require `UAV_project` to be started together with this workspace.
- Experiment 4 assumes ring center positions come from mocap; it does not use visual detection and does not close the loop on ring orientation.

# p600_arm_control

P600 无人机机械臂侧工作空间。这个仓库负责机械臂模型、机械臂控制、实验期望生成、录包与离线绘图，并通过统一话题与 [UAV_project](https://github.com/chnchenfan/P600_uav_control) 协同完成四个实验。

GitHub: <https://github.com/chnchenfan/p600_arm_control>  
配套飞控仓库: <https://github.com/chnchenfan/P600_uav_control>

## 基本信息

- 推荐运行环境: Ubuntu 18.04 + ROS Melodic
- 机械臂控制与实验节点位于 `src/arm_control`
- 消息、参数和可视化配置位于 `src/uam_message`
- 模型与 Gazebo 资源位于 `src/uam_v4`
- 脚本入口位于 `shell/`

核心目录:

- `src/arm_control/src`: 机械臂控制节点、实验 1/2/3/4 期望生成节点
- `src/arm_control/launch`: 仿真/实物启动入口
- `src/arm_control/shell`: 机械臂侧单独启动脚本
- `shell/Experiment`: 联合启动、录包脚本
- `shell/Plot`: 实验 1/2/3/4 离线绘图脚本

## 信息流

系统按“实验期望生成 -> 无人机执行 + 机械臂执行 -> 反馈闭环/记录”的方式工作。

1. 实验节点发布期望
- `uam_desired.cpp`
- `uam_desired_exp2.cpp`
- `uam_desired_exp3.cpp`
- `uam_desired_exp4.cpp`

这些节点统一提供：

- 服务 `/wjl/start/uav_desired`
- 无人机期望 `/wjl/guidefly/pose_d`
- 机械臂期望 `/wjl/arm/guidefly/angle_d`

2. 无人机侧执行
- `UAV_project` 中的 `desired_uav_fly.launch` 启动飞行控制链
- `UAV_project/src/uav/src/uav/Uav_info.cpp` 订阅 `/wjl/guidefly/pose_d`
- 无人机将 `x/y/z/yaw/land_flag` 转成 PX4/MAVROS 侧的飞行指令

3. 机械臂侧执行
- 真机: `rosrun arm_control serial_`
- 仿真: `motors_simulation`

执行节点消费 `/wjl/arm/guidefly/angle_d`，并反馈：

- `/wjl/arm/real/angle_r`
- `/wjl/arm/real/angle_error`

4. 反馈输入

实验节点按实验类型订阅以下反馈：

- `/mavros/local_position/pose`: 无人机基座实际位姿
- `/mavros/state`: 飞控连接与模式状态
- 动捕基座源按实验切换：
  - 实验 1 / 3: `/vrpn_client_node/Tracker0/pose`
  - 实验 2: `/vrpn_client_node/arm_base/pose`
  - 实验 4: `/vrpn_client_node/arm_target/pose`
- `/wjl/arm/real/angle_r`: 机械臂实际角
- 实验四额外使用 `/vrpn_client_node/ring1/pose` 到 `/vrpn_client_node/ring4/pose`

5. 数据记录与复盘

- `shell/Experiment/record_experiment1_data.sh`
- `shell/Experiment/record_experiment2_data.sh`
- `shell/Experiment/record_experiment3_data.sh`
- `shell/Experiment/record_experiment4_data.sh`

配套离线绘图:

- `shell/Plot/plot_experiment1.py`
- `shell/Plot/plot_experiment2.py`
- `shell/Plot/plot_experiment3.py`
- `shell/Plot/plot_experiment4.py`

## 四个实验

### 实验 1: 悬停下机械臂扰动基线

目标:
- 无人机保持固定悬停点
- 机械臂按双关节周期信号运动
- 观察基座跟踪与末端扰动响应

主要入口:
- 控制节点: `src/arm_control/src/uam_desired.cpp`
- 联合启动: `shell/Experiment/uam_control_desired.sh`
- 录包: `shell/Experiment/record_experiment1_data.sh`
- 绘图: `shell/Plot/plot_experiment1.py`

### 实验 2: `arm_base/arm_target` 全姿态末端定点补偿

目标:
- 无人机在 `x-z` 平面按圆弧参考运动
- `arm_base` 提供机械臂基座在动捕世界系下的完整位姿
- `arm_target` 提供末端工作点在动捕世界系下的实测位置
- 机械臂通过 2DoF 在线逆解与末端位置外环补偿，使末端尽量固定在世界系某一点

主要入口:
- 控制节点: `src/arm_control/src/uam_desired_exp2.cpp`
- 机械臂启动: `src/arm_control/shell/desired_uam_fly_exp2.sh`
- 联合启动: `shell/Experiment/uam_control_desired_exp2.sh`
- 录包: `shell/Experiment/record_experiment2_data.sh`
- 绘图: `shell/Plot/plot_experiment2.py`

### 实验 3: 基座方形轨迹 + 机械臂周期运动

目标:
- 无人机按正方形轨迹飞行
- 机械臂同步做双关节周期运动
- 加入基于 MAVROS 与动捕的一层保护逻辑

主要入口:
- 控制节点: `src/arm_control/src/uam_desired_exp3.cpp`
- 机械臂启动: `src/arm_control/shell/desired_uam_fly_exp3.sh`
- 联合启动: `shell/Experiment/uam_control_desired_exp3.sh`
- 录包: `shell/Experiment/record_experiment3_data.sh`
- 绘图: `shell/Plot/plot_experiment3.py`

### 实验 4: 空中穿环

目标:
- 无人机机械臂末端按顺序穿过 `ring1 -> ring2 -> ring3 -> ring4`
- 挂环位置由动捕实时提供
- 基座负责大范围搬运，机械臂负责局部对准
- 夹爪全程固定闭合，不包含抓取动作

主要入口:
- 控制节点: `src/arm_control/src/uam_desired_exp4.cpp`
- 机械臂启动: `src/arm_control/shell/desired_uam_fly_exp4.sh`
- 联合启动: `shell/Experiment/uam_control_desired_exp4.sh`
- 录包: `shell/Experiment/record_experiment4_data.sh`
- 绘图: `shell/Plot/plot_experiment4.py`

实验四额外输入:

- `/vrpn_client_node/ring1/pose`
- `/vrpn_client_node/ring2/pose`
- `/vrpn_client_node/ring3/pose`
- `/vrpn_client_node/ring4/pose`

## 真机启动顺序

统一顺序不变：

1. 启动动捕和 PX4
2. 启动对应实验的录包脚本
3. 启动对应实验的控制脚本

统一动捕入口：

- 实验 1: `bash shell/Experiment/uam_mocap.sh exp1`
- 实验 2: `bash shell/Experiment/uam_mocap.sh exp2`
- 实验 3: `bash shell/Experiment/uam_mocap.sh exp3`
- 实验 4: `bash shell/Experiment/uam_mocap.sh exp4`

这样会自动选择对应刚体：

- `exp1` / `exp3` -> `Tracker0`
- `exp2` -> `arm_base`
- `exp4` -> `arm_target`

## 常用入口

联合启动脚本:

- 实验 1: `shell/Experiment/uam_control_desired.sh`
- 实验 2: `shell/Experiment/uam_control_desired_exp2.sh`
- 实验 3: `shell/Experiment/uam_control_desired_exp3.sh`
- 实验 4: `shell/Experiment/uam_control_desired_exp4.sh`

仿真侧 launch:

- 实验 1: `src/arm_control/launch/simulation/desired_uam_fly.launch`
- 实验 2: `src/arm_control/launch/simulation/desired_uam_fly_exp2.launch`
- 实验 3: `src/arm_control/launch/simulation/desired_uam_fly_exp3.launch`
- 实验 4: `src/arm_control/launch/simulation/desired_uam_fly_exp4.launch`

## 说明

- 本仓库主要负责“机械臂与实验任务层”，不是单独的飞控仓库。
- 实验真正执行时，必须与 `UAV_project` 同时启动。
- 实验四默认依赖动捕返回挂环中心位置，不使用视觉识别，也不把挂环姿态纳入控制闭环。
