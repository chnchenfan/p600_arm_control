# uam_message

## English

`uam_message` provides the shared ROS messages, dynamic reconfigure definitions, YAML parameters, and plotting presets used by the UAV-arm system.

### Main contents

Messages:

- `msg/arm_angle.msg`
- `msg/pid.msg`
- `msg/uam_v4_pose.msg`

Dynamic reconfigure:

- `cfg/uam_cmd.cfg`
- `cfg/arm_pid.cfg`

Configuration and visualization files:

- `config/arm_info_init.yaml`
- `config/pid_set.xml`
- `config/uam_gazebo.xml`
- `config/uam_mocap.xml`

### Main interfaces

- `arm_angle.msg` is used for desired, measured, and error arm joint angles
- `uam_v4_pose.msg` is used by the model/visualization side
- `uam_cmd.cfg` and `arm_pid.cfg` support runtime tuning

### Role in the full system

This package is the shared data-definition layer. `arm_control`, `uav`, plotting scripts, and launch files all depend on it for common message and parameter definitions.

### Where it is used

- Required by `arm_control`
- Used by visualization and tuning tools
- Loaded by launch files through `arm_info_init.yaml`

## 中文

`uam_message` 提供无人机机械臂系统共用的 ROS 消息、自定义动态参数、YAML 参数文件以及可视化配置文件。

### 主要内容

消息:

- `msg/arm_angle.msg`
- `msg/pid.msg`
- `msg/uam_v4_pose.msg`

动态参数:

- `cfg/uam_cmd.cfg`
- `cfg/arm_pid.cfg`

配置与可视化文件:

- `config/arm_info_init.yaml`
- `config/pid_set.xml`
- `config/uam_gazebo.xml`
- `config/uam_mocap.xml`

### 主要接口

- `arm_angle.msg` 用于机械臂期望角、实测角和误差角
- `uam_v4_pose.msg` 用于模型/可视化相关数据
- `uam_cmd.cfg` 与 `arm_pid.cfg` 用于运行时调参

### 在整套系统中的角色

这个包是共享数据定义层。`arm_control`、`uav`、绘图脚本和多个 launch 文件都依赖这里的消息和参数定义。

### 使用场景

- 作为 `arm_control` 的依赖包
- 用于可视化和调参工具
- 在 launch 中通过 `arm_info_init.yaml` 加载初始参数
