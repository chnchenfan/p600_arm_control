# p600_arm_control

P600 无人机机械臂侧 catkin 工作空间。该仓库负责机械臂执行、机械臂/无人机实验期望生成、联合实验脚本、数据记录与离线绘图；飞控执行链位于配套仓库 [UAV_project](https://github.com/chnchenfan/P600_uav_control)。

- GitHub: <https://github.com/chnchenfan/p600_arm_control>
- 配套飞控仓库: <https://github.com/chnchenfan/P600_uav_control>
- 推荐环境: Ubuntu 18.04 + ROS Melodic

## 目录结构

- `src/arm_control`: 机械臂控制、实验期望生成、静态标定与仿真/实物 launch
- `src/uam_message`: 自定义消息、动态参数、机械臂初始参数和可视化配置
- `src/uam_v4`: UAV-arm 模型、URDF/SDF、Gazebo/RViz 启动文件
- `src/gazebo_plugin`: Gazebo 插件
- `src/xbox_control`: Xbox 手柄控制入口
- `shell/Compile`: 工作空间编译脚本
- `shell/Experiment`: 真机/联合实验启动、录包和静态标定采集脚本
- `shell/Simulation`: 仿真启动脚本
- `shell/Plot`: 实验数据离线绘图脚本

## 与 UAV_project 的分工

本仓库是“机械臂与任务层”，负责发布实验期望：

- `/wjl/guidefly/pose_d`: 无人机位置、偏航和降落标志
- `/wjl/arm/guidefly/angle_d`: 机械臂关节期望
- `/wjl/start/uav_desired`: 启动实验期望生成的服务

`UAV_project` 是“飞控执行层”，负责接收 `/wjl/guidefly/pose_d`，通过 PX4/MAVROS 执行无人机运动，并反馈 `/mavros/local_position/pose`、`/mavros/state` 等状态。

机械臂执行节点消费 `/wjl/arm/guidefly/angle_d`，并发布：

- `/wjl/arm/real/angle_d`: 执行层目标角
- `/wjl/arm/real/angle_r`: 机械臂实际角
- `/wjl/arm/real/angle_error`: 机械臂关节误差

## 编译

脚本内默认使用 `~/p600_arm_control` 和 `~/UAV_project`。如果仓库不在 home 目录下，建议先建立软链接或按实际路径手动进入工作空间编译。

```bash
cd ~/p600_arm_control
bash shell/Compile/catkin_make_all.sh
source devel/setup.bash
```

编译顺序为：

1. `uam_message`
2. `arm_control`
3. `xbox_control`

`arm_control` 依赖 `UAV_project/devel/include` 中的无人机消息/服务头文件，因此通常需要先编译 `UAV_project` 的 `uav` 包。

## 四个实验

### 实验 1: 悬停下机械臂扰动基线

目标：

- 无人机保持固定悬停点
- 机械臂执行双关节周期运动
- 观察机械臂扰动下的基座跟踪与末端响应

入口：

- 实验节点: `src/arm_control/src/uam_desired.cpp`
- 联合启动: `bash shell/Experiment/uam_control_desired.sh`
- 录包: `bash shell/Experiment/record_experiment_data.sh exp1`
- 绘图: `python3 shell/Plot/plot_experiment1.py`

### 实验 2: 基座运动下的末端定点补偿

目标：

- 无人机基座按实验参考运动
- 动捕提供机械臂基座位姿和末端工作点反馈
- 机械臂通过在线逆解与补偿，使末端尽量保持在世界系固定点附近

入口：

- 实验节点: `src/arm_control/src/uam_desired_exp2.cpp`
- 联合启动: `bash shell/Experiment/uam_control_desired_exp2.sh`
- 静态 IK 测试: `bash shell/Experiment/uam_control_desired_exp2_static_ik.sh`
- 验证入口: `bash shell/Experiment/uam_control_desired_exp2_validation.sh`
- 录包: `bash shell/Experiment/record_experiment_data.sh exp2`
- 绘图: `python3 shell/Plot/plot_experiment2.py`
- 标定说明: `src/arm_control/README_calibration.md`

### 实验 3: 基座方形轨迹 + 机械臂周期运动

目标：

- 无人机跟踪方形轨迹
- 机械臂同步执行周期运动
- 实验节点使用 MAVROS 和动捕反馈进行状态判断与保护

入口：

- 实验节点: `src/arm_control/src/uam_desired_exp3.cpp`
- 联合启动: `bash shell/Experiment/uam_control_desired_exp3.sh`
- 录包: `bash shell/Experiment/record_experiment_data.sh exp3`
- 绘图: `python3 shell/Plot/plot_experiment3.py`

### 实验 4: 空中穿环

目标：

- 末端按顺序穿过 `ring1 -> ring2 -> ring3 -> ring4`
- 挂环中心位置由动捕实时提供
- 无人机基座负责大范围运输，机械臂负责局部对准
- 夹爪全程保持闭合，不包含视觉识别或抓取逻辑

入口：

- 实验节点: `src/arm_control/src/uam_desired_exp4.cpp`
- 联合启动: `bash shell/Experiment/uam_control_desired_exp4.sh`
- 录包: `bash shell/Experiment/record_experiment_data.sh exp4`
- 绘图: `python3 shell/Plot/plot_experiment4.py`

实验 4 额外依赖动捕话题：

- `/vrpn_client_node/ring1/pose`
- `/vrpn_client_node/ring2/pose`
- `/vrpn_client_node/ring3/pose`
- `/vrpn_client_node/ring4/pose`

## 常用启动

### 编译

```bash
cd ~/p600_arm_control
bash shell/Compile/catkin_make_all.sh
```

### 数据记录

```bash
cd ~/p600_arm_control
source devel/setup.bash
bash shell/Experiment/record_experiment_data.sh exp1
```

示例输出：

```text
实验录包输出目录: /home/amov/p600_arm_control/data/exp1/20260514_001234
bag文件: /home/amov/p600_arm_control/data/exp1/20260514_001234/exp1_px4.bag
```

### 一、实验一

1. 启动动捕和 PX4：

```bash
cd ~/p600_arm_control
bash shell/Experiment/uam_mocap.sh exp1 192.168.xxx.xxx
```

2. 启动控制实验：

```bash
cd ~/p600_arm_control
bash shell/Experiment/uam_control_desired.sh
```

### 二、实验二

```bash
cd ~/p600_arm_control
bash shell/Experiment/uam_mocap.sh exp2 192.168.xxx.xxx
bash shell/Experiment/uam_control_desired_exp2.sh
```

机械臂数据标定与拟合：

```bash
cd ~/p600_arm_control
bash shell/Experiment/uam_mocap.sh exp2 <VRPN_SERVER_IP>
bash shell/Experiment/uam_static_calibration_collect.sh
bash shell/Experiment/record_static_calibration_data.sh
```

后续在自己电脑上绘图：

```bash
source /opt/ros/noetic/setup.bash
python3 /home/cf/Program/code/P600_uam/p600_arm_control/shell/Calibration/fit_static_geometry_stage1.py --bag /home/cf/Program/code/P600_uam/data/calibration/static/2026324_1/calibration_static.bag
```

逆解静态测试：

```bash
cd ~/p600_arm_control
bash shell/Experiment/uam_mocap.sh exp2 <VRPN_SERVER_IP>
```

逆解原位静态测试：

```bash
bash shell/Experiment/uam_control_desired_exp2_static_ik.sh
```

逆解多组静态测试，包含飞机虚拟扰动：

```bash
bash shell/Experiment/uam_control_desired_exp2_validation.sh
```

### 三、实验三

```bash
cd ~/p600_arm_control
bash shell/Experiment/uam_mocap.sh exp3 192.168.228.99
bash shell/Experiment/uam_control_desired_exp3.sh
```

### 四、实验四

```bash
cd ~/p600_arm_control
bash shell/Experiment/uam_mocap.sh exp4 <VRPN_SERVER_IP>
bash shell/Experiment/uam_control_desired_exp4.sh
```

## 注意事项

- 真机实验需要同时启动本仓库与 `UAV_project`。
- 多个脚本默认使用 `~/p600_arm_control` 和 `~/UAV_project`，换路径部署时需要同步调整或创建软链接。
- 实验 4 只使用动捕提供的挂环中心位置，不使用视觉检测，也不闭环控制挂环姿态。
- `record_experiment_data.sh` 会提示缺失话题，但不会因为某个话题暂时不存在而中止录包。
