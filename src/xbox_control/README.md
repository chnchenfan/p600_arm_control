# xbox_control

## English

`xbox_control` is a manual joystick control package used for debugging and manual arm command injection. It reads joystick input and publishes arm angle commands to the standard guideflight arm topic.

### Main executable and launch

- Executable: `xbox_control`
- Launch: `launch/xbox_control_start.launch`
- Helper script: `shell/xbox_start.sh`

### Main interfaces

Published:

- `/wjl/arm/guidefly/angle_d`

Used together with:

- `joy/joy_node`
- `arm_control/serial_`
- `arm_control/motors_simulation`

### Role in the full system

This package is a manual debug tool. It is not part of the automatic experiment pipeline, but it can override or directly provide arm commands during integration testing.

## 中文

`xbox_control` 是一个用于调试和手动注入机械臂指令的摇杆控制包。它读取手柄输入，并将机械臂角度命令发布到标准机械臂控制话题。

### 主要节点与启动入口

- 可执行节点: `xbox_control`
- Launch: `launch/xbox_control_start.launch`
- 辅助脚本: `shell/xbox_start.sh`

### 主要接口

发布:

- `/wjl/arm/guidefly/angle_d`

通常配合:

- `joy/joy_node`
- `arm_control/serial_`
- `arm_control/motors_simulation`

### 在整套系统中的角色

这个包主要用于手工调试，不属于自动实验主流程，但在联调阶段可以直接给机械臂发送控制命令。
