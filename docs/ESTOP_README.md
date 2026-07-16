# 急停 (E-Stop) 方案

## 架构概览

```
Topic: /robot_hardware_estop/estop (std_msgs/Bool)
    │
    ▼
RobotHardwareNode 独立 spin 线程
    │  estop_flag_.store(msg->data)
    ▼
write() 每个控制周期检查 estop_flag_
    │  if true → joint.motor->estop(joint.estop_kd)
    │  if false → 正常控制器力矩命令
    ▼
MotorDriver::estop(kd)  (虚函数，各驱动自行实现)
    │  motor_mit_cmd(0, 0, 0, kd, 0)  // kp=0, v_des=0, torque=0
    ▼
电机内部 MIT 闭环: T = kd*(0 - v) = -kd*v → 纯速度阻尼
```

**核心思路：** 急停不使用控制器（控制器不感知急停），而是直接在 `write()` 层拦截，利用电机 MIT 模式的内部 `kd` 阻尼快速减速到零。阻尼在电机 MCU 上以 kHz 级闭环，不受外部 250Hz 控制周期和 CAN 延迟影响。

## 移植步骤

### 1. 基类 `motor_driver.hpp` — 加纯虚函数

```cpp
/**
 * @brief 急停：使用电机 MIT 模式内部 kd 阻尼，快速减速到零。
 * 每个控制周期调用一次，不是一次性命令。
 * kp=0, torque=0, 目标速度=0，仅靠 kd 产生阻尼力矩。
 * @param kd 阻尼系数（各驱动按自身 MIT 参数范围解释）
 */
virtual void estop(float kd) = 0;
```

### 2. 各电机驱动 — 实现 `estop()`

```cpp
// DM 驱动 (dm_motor_driver.cpp)
void DmMotorDriver::estop(float kd) {
    motor_mit_cmd(0.0f, 0.0f, 0.0f, kd, 0.0f);
}

// EVO 驱动 (evo_motor_driver.cpp)
void EvoMotorDriver::estop(float kd) {
    motor_mit_cmd(0.0f, 0.0f, 0.0f, kd, 0.0f);
}

// ENCOS 驱动 (encos_motor_driver.cpp)
void EncosMotorDriver::estop(float kd) {
    motor_mit_cmd(0.0f, 0.0f, 0.0f, kd, 0.0f);
}

// STW 驱动 (stw_motor_driver.cpp)
void StwMotorDriver::estop(float kd) {
    motor_mit_cmd(0.0f, 0.0f, 0.0f, kd, 0.0f);
}
```

> **注意：** 某些 DM 电机型号在 `kp=0` 时会关闭 MIT 回路导致 `kd` 不生效。如果硬编码 kd 有效但从配置读取无效，检查第 4 步的 xacro 参数传递。如果参数传递正确但仍无阻尼，尝试给 kp 一个极小值（如 0.05）：
> ```cpp
> void DmMotorDriver::estop(float kd) {
>     motor_mit_cmd(motor_pos_.load(), 0.0f, 0.05f, kd, 0.0f);
> }
> ```

### 3. 关节配置 YAML — 加 `estop_kd` 参数

```yaml
joints:
  - name: neck_yaw_joint
    motor_id: 3
    interface: can0
    interface_type: can
    motor_type: DM
    motor_model: 2
    master_id_offset: 1
    direction: 1.0
    zero_offset: 0.0
    estop_kd: 0.5          # ← 急停阻尼系数，各关节可独立配置
```

> 暂设为 0 不影响正常运行。实测后按需调整：值越大刹车越快，但过大可能引起震荡。DM 电机建议从 0.5 起步。

### 4. 关键：xacro 必须传递 `estop_kd`

**这一步最容易遗漏。** 在 `ros2control.xacro` 的 joint 宏里加上：

```xml
<xacro:macro name="your_ros2_control_joint" params="joint">
    <joint name="${joint.name}">
      <!-- ... 其他参数 ... -->
      <param name="motor_zero_offset">${joint.zero_offset}</param>
      <param name="estop_kd">${joint.estop_kd}</param>        <!-- ← 必须加 -->
      <command_interface name="effort"/>
      <!-- ... -->
    </joint>
</xacro:macro>
```

数据链路：`YAML → xacro → URDF → ros2_control → on_init() → joint.estop_kd`。xacro 是中间唯一的手动传递环节，漏掉会导致 `estop_kd` 永远是默认值 0。

### 5. `robot_hardware_node.hpp` — 加成员

```cpp
#include <atomic>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

// JointData 结构体加字段
struct JointData {
    // ...
    float estop_kd{0.0f};  // 急停阻尼系数
};

// 类私有成员
std::atomic<bool> estop_flag_{false};
rclcpp::Node::SharedPtr estop_node_;
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
rclcpp::executors::SingleThreadedExecutor::SharedPtr estop_executor_;
std::unique_ptr<std::thread> estop_spin_thread_;
```

### 6. `robot_hardware_node.cpp` — 初始化、订阅、拦截

#### 6.1 `on_init()` — 读取配置

```cpp
// 照 direction、zero_offset 的方式加
joint.estop_kd = static_cast<float>(
    get_joint_double_param(joint_info, "estop_kd", 0.0));
```

#### 6.2 `on_configure()` — 创建急停订阅

```cpp
// 在硬件驱动创建完成后，return SUCCESS 之前
estop_node_ = std::make_shared<rclcpp::Node>("robot_hardware_estop");
estop_sub_ = estop_node_->create_subscription<std_msgs::msg::Bool>(
    "~/estop", 10,
    [this](std_msgs::msg::Bool::SharedPtr msg) {
        estop_flag_.store(msg->data);
    });
estop_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
estop_executor_->add_node(estop_node_);
estop_spin_thread_ = std::make_unique<std::thread>(
    [this]() { estop_executor_->spin(); });
```

#### 6.3 `on_deactivate()` — 清理

```cpp
estop_flag_.store(false);
if (estop_executor_) {
    estop_executor_->cancel();
}
if (estop_spin_thread_ && estop_spin_thread_->joinable()) {
    estop_spin_thread_->join();
}
estop_spin_thread_.reset();
estop_sub_.reset();
estop_node_.reset();
estop_executor_.reset();
```

#### 6.4 `write()` — 急停拦截

```cpp
hardware_interface::return_type RobotHardwareNode::write(...) {
    const bool estop_active = estop_flag_.load();

    for_each_bus_parallel([this, estop_active](const BusData & bus) {
        for (const auto joint_index : bus.joint_indices) {
            auto & joint = joints_[joint_index];
            if (!joint.motor || !joint.active) continue;

            // 急停优先：替换控制器力矩命令
            if (estop_active) {
                joint.motor->estop(joint.estop_kd);
                continue;
            }

            // 正常控制逻辑（原有代码）...
        }
    });
}
```

## 使用方式

### 触发急停

```bash
ros2 topic pub /robot_hardware_estop/estop std_msgs/msg/Bool "{data: true}" -1
```

### 解除急停

```bash
ros2 topic pub /robot_hardware_estop/estop std_msgs/msg/Bool "{data: false}" -1
```

### 程序中触发

```python
import rclpy
from std_msgs.msg import Bool

node = rclpy.create_node('estop_trigger')
pub = node.create_publisher(Bool, '/robot_hardware_estop/estop', 10)

# 急停
pub.publish(Bool(data=True))

# 解除
pub.publish(Bool(data=False))
```

## 设计要点

| 特性 | 说明 |
|------|------|
| 优先级 | 急停 > 控制器命令，`write()` 第一件事就是检查 estop_flag_ |
| 控制器无感知 | 急停不改变控制器状态，控制器继续运行，只是输出被忽略 |
| 解除后即时恢复 | `estop_flag_` 变 false 后下一周期即恢复正常控制 |
| 线程安全 | `estop_flag_` 是 `std::atomic<bool>`，topic 回调和 write() 分属不同线程 |
| 独立 spin | 使用独立 Node + SingleThreadedExecutor + 独立线程，不影响 ros2_control 主循环 |
| 周期调用 | `estop()` 每个控制周期都调用，不是一次性命令，防止丢帧 |
| 可移植性 | 每种电机驱动自行实现 `estop()`，新加电机类型只需实现自己的急停策略 |

## 配置调参

- `estop_kd: 0.0` → 急停时只发零力矩，无主动阻尼（等同于之前的 on_deactivate 行为）
- `estop_kd: 0.5` → 轻度阻尼，减速平缓
- `estop_kd: 2.0` → 较强阻尼，快速刹车
- `estop_kd: 5.0+` → 强阻尼，DM 电机 OKdMax=5，超过会被 clamp

建议各关节独立配置：大惯量关节（如腰部）给较大 kd，小惯量关节给较小 kd。
