# T002 机器人开发日志 — 2025-07-06

## 一、数据流架构

```
手柄 xbox_text  →  /target_pose (8 values)
    ↓
torque_control_node
    │  读 /joint_states → 脖子 FK+PD+Jacobian → 力矩
    │  读 target_vx/vy/wz → 底盘运动学 → 转向角+轮速
    ↓
/t002_controller/command (8 values)
    ↓
t002_controller (C++)
    │  Torque模式: 透传  Velocity模式: PD  Position模式: PD
    ↓
GazeboSimSystem / RobotHardwareNode
    ↓
电机 (仿真/真实)
```

## 二、今日问题与解决

### 1. Gazebo 所有关节 Not existing
**现象**: `prepare command mode switch was rejected`, 全部 8 个 effort 接口不存在
**原因**: Gazebo xacro 没传 `hardware_plugin="gz_ros2_control/GazeboSimSystem"`, 走了默认值 `robot_hardware/RobotHardwareNode`
**解决**: xacro 里加 `hardware_plugin="gz_ros2_control/GazeboSimSystem"` 参数
**教训**: 每次重新生成 xacro 都要检查这个参数

### 2. joint_state_broadcaster 激活但不发布
**现象**: `Configured and activated` 但 `/joint_states` 空
**原因1**: `interfaces: [position, velocity]` 参数和 broadcaster 内部逻辑冲突
**解决1**: 删掉 `interfaces`, 只保留 `joints`
**原因2**: `velocity="0"` 在 joint limit 中 → Gazebo 锁死所有关节 → 无状态数据
**解决2**: 改为 `velocity="50"`

### 3. 控制器 command_callback 钳位力矩
**现象**: 脖子力矩被掐到 ±0.35
**原因**: `sanitize_position()` 对所有关节都按位置限制钳位, torque 模式不该钳
**解决**: 在 `command_callback` 中判断 `JointMode::Torque` 跳过钳位

### 4. 机器人不动 / 力矩太小
**现象**: command 正确 (effort=-5Nm) 但关节不动, joint_states effort 只有 0.02
**原因**: SolidWorks 导出所有关节 `velocity="0"` → Gazebo 物理锁死
**解决**: 源URDF中 `velocity="0"` → `velocity="50"`, 重新生成 xacro

### 5. PD 参数调优过程
- 初始: kp=0.2 → 力矩 0.007 Nm (太小)
- 放大: kp=5.0 → 有效
- 放大: kp=100 (×20) → 过强
- 最终: kp=5.0, kd=0.1 (关节PD), kp=5.0 (姿态PD)

### 6. 轮子速度模式
**设计**: wheel-left-roll-joint 和 wheel-right-roll-joint → velocity 模式
- 控制器: `effort = Kp * (cmd_vel - actual_vel)` (Kp=1.0)
- 底盘运动学: RA/RB → atan2(方向) + hypot/radius (速度)
- 最短路径翻转: 角度差 > π/2 → 目标+π, 速度取反
- 死区: 角度差 < 0.6 rad → 速度减半

## 三、开发规范

### 文件约定
```
src/controller/t002_controller/   ← C++ 控制器, 不改硬件
src/head_solver/                  ← Python 解算节点
src/hardware/robot_hardware/      ← 硬件抽象层, 不动
src/hardware/motors/              ← 电机驱动, 不动
urdfs/T002_description/           ← URDF + 配置文件
```

### 命名规范
- 关节名统一用下划线: `neck_pitch_joint`, `wheel_left_roll_joint`
- URDF 和 controller.yaml 关节名严格一致
- 脖子=力矩模式 (torque), 轮子转向=位置模式 (position), 轮子滚动=速度模式 (velocity)

### xacro 生成
- 源: `/home/tuf/桌面/总装配体urdf/urdf/总装配体urdf.urdf`
- 真机: `T002_description.urdf.xacro` (plugin=RobotHardwareNode)
- Gazebo: `T002_gazebo_des.urdf.xacro` (plugin=GazeboSimSystem + hardware_plugin 参数)
- 源 URDF 变了就运行: `python3 urdfs/regenerate_xacros.py`

### YAML 参数格式
- `joint_modes`: 字符串列表 (position/velocity/torque)
- `pd`, `joint_limits`, `effort_limits`: `;` 分号分隔字符串, C++ parse_semicolon_list 解析
- biome_state_broadcaster: 只配 `joints`, 不要 `interfaces`

### 调试流程
1. `ros2 topic list` 确认话题存在
2. `ros2 topic echo /joint_states` 看关节状态
3. `ros2 topic echo /t002_controller/command` 看命令值
4. `ros2 topic echo /target_pose` 看手柄输入
5. Gazebo 日志看 `Loading joint:` 是否全加载

### 关键参数
| 参数 | 位置 | 值 |
|------|------|-----|
| wheel_base | torque_control_node | 0.44553 m |
| wheel_radius | torque_control_node | 0.125 m |
| neck kp/kd | torque_control_node | 5.0 / 0.0 |
| waist kp/kd | controller.yaml | 5.0 / 0.1 |
| wheel yaw kp/kd | controller.yaml | 5.0 / 0.1 |
| wheel roll kp | controller.yaml | 1.0 (vel) |
| neck torque limits | controller.yaml | ±1.5 Nm |
| max_motor_torque | torque_control_node | 1.5 Nm |



### 仿真用到的参数
   joint_limits:
        - -3.14; 3.14; -3.14; 3.14; -0.35; 0.35; -0.35; 0.35
        - -6.28; 6.28; -50.0; 50.0; -6.28; 6.28; -50.0; 50.0
      pd:
        - 5.0; 0.1; 5.0; 0.1; 0.0; 0.0; 0.0; 0.0
        - 5.0; 0.1; 1.0; 0.0; 5.0; 0.1; 1.0; 0.0
      effort_limits:
        - -5.0; 5.0; -5.0; 5.0; -1.5; 1.5; -1.5; 1.5
        - -5.0; 5.0; -5.0; 5.0; -5.0; 5.0; -5.0; 5.0
      default_joint_positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
