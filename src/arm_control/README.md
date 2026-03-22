# arm_control

## English

`arm_control` is the main arm-side execution and experiment package in `p600_arm_control`. It contains the physical arm driver, the simulation arm driver, a dynamic manual reference publisher, and the dedicated experiment nodes for experiments 1 to 4.

### Main executables

- `serial_`: physical arm serial driver
- `motors_simulation`: simulated arm actuator bridge
- `dynamic_angle`: manual arm/UAV reference publisher
- `uam_desired`: experiment 1 reference generator
- `uam_desired_exp2`: experiment 2 reference generator with `arm_base/arm_target` full-pose compensation
- `uam_desired_exp3`: experiment 3 reference generator
- `uam_desired_exp4`: experiment 4 reference generator

### Main launch and script entrypoints

- `launch/physical/arm_physical_control.launch`
- `launch/simulation/desired_uam_fly.launch`
- `launch/simulation/desired_uam_fly_exp2.launch`
- `launch/simulation/desired_uam_fly_exp3.launch`
- `launch/simulation/desired_uam_fly_exp4.launch`
- `launch/simulation/uam_guidefly.launch`
- `shell/desired_uam_fly.sh`
- `shell/desired_uam_fly_exp2.sh`
- `shell/desired_uam_fly_exp3.sh`
- `shell/desired_uam_fly_exp4.sh`

### Main interfaces

Published:

- `/wjl/arm/guidefly/angle_d`
- `/wjl/guidefly/pose_d`
- `/wjl/arm/real/angle_r`
- `/wjl/arm/real/angle_error`

Consumed:

- `/vrpn_client_node/arm_base/pose`
- `/vrpn_client_node/arm_target/pose`
- `/mavros/local_position/pose`
- `/mavros/state`
- `/wjl/arm/real/angle_r`
- `/vrpn_client_node/ring1/pose` to `/vrpn_client_node/ring4/pose` in experiment 4

Service:

- `/wjl/start/uav_desired`

### Role in the full system

This package is the task layer for the UAV-arm experiments. It computes arm and UAV references and hands the UAV part to `UAV_project` through `/wjl/guidefly/pose_d`.

### Where it is used

- Used directly during real experiments together with `UAV_project`
- Used in Gazebo-based simulation with `motors_simulation`
- Used for experiments 1 to 4 and for manual debugging through `dynamic_angle`

## 中文

`arm_control` 是 `p600_arm_control` 中最核心的机械臂执行与实验任务包，包含真机机械臂驱动、仿真机械臂驱动、动态手工期望发布，以及实验 1 到实验 4 的专用控制节点。

### 主要可执行节点

- `serial_`: 真机机械臂串口驱动
- `motors_simulation`: 仿真机械臂执行桥接
- `dynamic_angle`: 手工机械臂/无人机期望发布节点
- `uam_desired`: 实验 1 期望生成节点
- `uam_desired_exp2`: 实验 2 期望生成节点
- `uam_desired_exp3`: 实验 3 期望生成节点
- `uam_desired_exp4`: 实验 4 期望生成节点

### 主要 launch 与脚本入口

- `launch/physical/arm_physical_control.launch`
- `launch/simulation/desired_uam_fly.launch`
- `launch/simulation/desired_uam_fly_exp2.launch`
- `launch/simulation/desired_uam_fly_exp3.launch`
- `launch/simulation/desired_uam_fly_exp4.launch`
- `launch/simulation/uam_guidefly.launch`
- `shell/desired_uam_fly.sh`
- `shell/desired_uam_fly_exp2.sh`
- `shell/desired_uam_fly_exp3.sh`
- `shell/desired_uam_fly_exp4.sh`

### 主要接口

发布:

- `/wjl/arm/guidefly/angle_d`
- `/wjl/guidefly/pose_d`
- `/wjl/arm/real/angle_r`
- `/wjl/arm/real/angle_error`

订阅:

- `/vrpn_client_node/arm_base/pose`
- `/vrpn_client_node/arm_target/pose`
- `/mavros/local_position/pose`
- `/mavros/state`
- `/wjl/arm/real/angle_r`
- 实验四中的 `/vrpn_client_node/ring1/pose` 到 `/vrpn_client_node/ring4/pose`

服务:

- `/wjl/start/uav_desired`

### 在整套系统中的角色

这个包是无人机机械臂实验的任务层，负责计算机械臂和无人机期望，并通过 `/wjl/guidefly/pose_d` 将无人机部分交给 `UAV_project` 执行。

### 使用场景

- 真机实验时与 `UAV_project` 配合使用
- Gazebo 仿真时通过 `motors_simulation` 使用
- 用于实验 1 到实验 4，以及 `dynamic_angle` 手工调试
