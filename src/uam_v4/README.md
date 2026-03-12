# uam_v4

## English

`uam_v4` is the UAV-arm model description package. It stores the URDF/SDF model, mesh assets, joint-name configuration, and basic display/Gazebo launch files for the P600 UAV-arm platform.

### Main contents

- `urdf/uam_v4.urdf`
- `urdf/uam_v4.sdf`
- `urdf/uam_v4.csv`
- `meshes/`
- `config/joint_names_uam_v4.yaml`
- `launch/display.launch`
- `launch/gazebo.launch`

### Role in the full system

This package is the model asset layer. It does not generate task references. Instead, it provides the geometry and robot-description resources used by simulation, visualization, and related tooling.

### Where it is used

- Used by Gazebo simulation
- Used by RViz / display launch flows
- Referenced by other packages that need the UAV-arm robot model

## 中文

`uam_v4` 是无人机机械臂整机模型描述包，存放 P600 平台的 URDF/SDF 模型、网格资源、关节名称配置以及基础显示与 Gazebo 启动文件。

### 主要内容

- `urdf/uam_v4.urdf`
- `urdf/uam_v4.sdf`
- `urdf/uam_v4.csv`
- `meshes/`
- `config/joint_names_uam_v4.yaml`
- `launch/display.launch`
- `launch/gazebo.launch`

### 在整套系统中的角色

这个包是模型资源层，不负责实验任务生成，而是为仿真、可视化和其他依赖包提供统一的机器人描述资源。

### 使用场景

- 用于 Gazebo 仿真
- 用于 RViz / 模型显示流程
- 被其他需要 UAV-arm 模型描述的包引用
