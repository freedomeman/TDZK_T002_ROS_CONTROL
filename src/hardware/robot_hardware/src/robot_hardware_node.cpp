#include "robot_hardware_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace robot_hardware
{

// ============================================================================
// 生命周期回调: on_init
// 负责解析 URDF/参数 中定义的 joint 和 sensor 配置，构建内部数据结构，
// 并创建 CAN 总线分组。当系统第一次加载该硬件接口时调用。
// ============================================================================
hardware_interface::CallbackReturn RobotHardwareNode::on_init(
  const hardware_interface::HardwareInfo & hardware_info)
{
  // 调用父类(SystemInterface)的 on_init，进行通用的初始化(如解析 info_)
  if (hardware_interface::SystemInterface::on_init(hardware_info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
  rclcpp::get_logger("robot_hardware_node"),
  "硬件 抽象层初始化开始 -------------------------------------------------------------");

  // ----- 处理所有关节(joint) -----
  joints_.clear();
  joints_.reserve(info_.joints.size());  // 预分配内存，避免多次扩容

  try {
    for (std::size_t index = 0; index < info_.joints.size(); ++index) {
      const auto & joint_info = info_.joints[index];
      JointData joint;                               // 新建一个关节数据结构
      joint.name = joint_info.name;                  // 关节名称

      // 从关节参数中读取必要的字段 (这些参数定义在 URDF 或 ros2_control 的 <param> 标签中)
      joint.interface       = get_required_joint_param(joint_info, "interface");       // 接口名称(如 can0)
      joint.interface_type  = get_required_joint_param(joint_info, "interface_type");  // 接口类型(如 "can")
      joint.motor_type      = get_required_joint_param(joint_info, "motor_type");      // 电机类型(如 "dm_motor")
      joint.motor_model     = get_required_joint_int_param(joint_info, "motor_model"); // 电机型号ID
      joint.master_id_offset = static_cast<uint16_t>(
        get_required_joint_int_param(joint_info, "master_id_offset"));                 // 主站ID偏移
      joint.motor_id         = static_cast<uint16_t>(get_required_joint_int_param(joint_info, "motor_id")); // 电机CAN ID
      joint.zero_offset      = get_required_joint_double_param(joint_info, "motor_zero_offset"); // 零点偏移
      joint.direction        = get_required_joint_double_param(joint_info, "direction");        // 力矩方向系数(1或-1)
      joint.estop_kd         = static_cast<float>(
        get_joint_double_param(joint_info, "estop_kd", 0.0));  // 急停阻尼系数
      // 为状态和命令接口分配存储空间，初始化为 NaN 或 0
      joint.state_values.resize(
        joint_info.state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());
      joint.command_values.resize(joint_info.command_interfaces.size(), 0.0);

      // 使用辅助函数 cache_state_interface 查找指定名称的状态接口，并记录其索引和是否存在
      const auto cache_state_interface =
        [&joint, &joint_info](const std::string & interface_name, std::size_t & index, bool & exists) {
          const auto iter = std::find_if(
            joint_info.state_interfaces.begin(),
            joint_info.state_interfaces.end(),
            [&interface_name](const auto & interface) {
              return interface.name == interface_name;
            });
          if (iter != joint_info.state_interfaces.end()) {
            index = static_cast<std::size_t>(
              std::distance(joint_info.state_interfaces.begin(), iter));
            exists = true;
          }
        };
      // 缓存位置和速度状态接口
      cache_state_interface(
        hardware_interface::HW_IF_POSITION,
        joint.position_state_index,
        joint.has_position_state);
      cache_state_interface(
        hardware_interface::HW_IF_VELOCITY,
        joint.velocity_state_index,
        joint.has_velocity_state);

      // 查找力矩命令接口 (effort)
      const auto effort_command_iter = std::find_if(
        joint_info.command_interfaces.begin(),
        joint_info.command_interfaces.end(),
        [](const auto & interface) {
          return interface.name == hardware_interface::HW_IF_EFFORT;
        });
      if (effort_command_iter != joint_info.command_interfaces.end()) {
        joint.effort_command_index = static_cast<std::size_t>(
          std::distance(joint_info.command_interfaces.begin(), effort_command_iter));
        joint.has_effort_command = true;
      }

      joints_.push_back(std::move(joint));  // 移动构造，避免拷贝大结构体
    }

    // ----- 处理所有传感器(sensor) -----
    sensors_.clear();
    sensors_.reserve(info_.sensors.size());
    for (std::size_t index = 0; index < info_.sensors.size(); ++index) {
      const auto & sensor_info = info_.sensors[index];
      SensorData sensor;
      sensor.name = sensor_info.name;
      // 传感器同样需要接口参数
      sensor.interface      = get_required_joint_param(sensor_info, "interface");
      sensor.interface_type = get_required_joint_param(sensor_info, "interface_type");
      sensor.imu_type       = get_required_joint_param(sensor_info, "imu_type");
      sensor.baudrate       = get_required_joint_int_param(sensor_info, "baudrate");
      sensor.state_values.resize(
        sensor_info.state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());

      // 缓存 IMU 四元数、角速度、线性加速度各个分量对应的状态接口
      const auto cache_sensor_interface =
        [&sensor, &sensor_info](
          const std::string & interface_name,
          std::size_t & index,
          bool & exists) {
          const auto iter = std::find_if(
            sensor_info.state_interfaces.begin(),
            sensor_info.state_interfaces.end(),
            [&interface_name](const auto & interface) {
              return interface.name == interface_name;
            });
          if (iter != sensor_info.state_interfaces.end()) {
            index = static_cast<std::size_t>(
              std::distance(sensor_info.state_interfaces.begin(), iter));
            exists = true;
          }
        };
      cache_sensor_interface(
        "orientation.w", sensor.orientation_w_index, sensor.has_orientation_w);
      cache_sensor_interface(
        "orientation.x", sensor.orientation_x_index, sensor.has_orientation_x);
      cache_sensor_interface(
        "orientation.y", sensor.orientation_y_index, sensor.has_orientation_y);
      cache_sensor_interface(
        "orientation.z", sensor.orientation_z_index, sensor.has_orientation_z);
      cache_sensor_interface(
        "angular_velocity.x", sensor.angular_velocity_x_index, sensor.has_angular_velocity_x);
      cache_sensor_interface(
        "angular_velocity.y", sensor.angular_velocity_y_index, sensor.has_angular_velocity_y);
      cache_sensor_interface(
        "angular_velocity.z", sensor.angular_velocity_z_index, sensor.has_angular_velocity_z);
      cache_sensor_interface(
        "linear_acceleration.x",
        sensor.linear_acceleration_x_index,
        sensor.has_linear_acceleration_x);
      cache_sensor_interface(
        "linear_acceleration.y",
        sensor.linear_acceleration_y_index,
        sensor.has_linear_acceleration_y);
      cache_sensor_interface(
        "linear_acceleration.z",
        sensor.linear_acceleration_z_index,
        sensor.has_linear_acceleration_z);

      sensors_.push_back(std::move(sensor));
    }
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      rclcpp::get_logger("robot_hardware_node"),
      "硬件参数配置错误: %s",
      exception.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 根据关节的 interface 字段将它们分组到不同的 CAN 总线上
  rebuild_bus_groups();
  // 如果有多条总线，创建线程池用于后续并行读写
  if (buses_.size() > 1) {
    bus_thread_pool_ = std::make_unique<ThreadPool>(buses_.size());
  } else {
    bus_thread_pool_.reset();
  }


  RCLCPP_INFO(
    rclcpp::get_logger("robot_hardware_node"),
    "硬件 抽象层初始化完成  %zu joints on %zu CAN buses, %zu sensors. -------------------------------------------------------------",
    joints_.size(),
    buses_.size(),
    sensors_.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// 导出状态接口 (state interfaces)
// 将 joints_ 和 sensors_ 中 state_values 的地址暴露给 ros2_control 框架，
// 框架会通过此指针直接读取硬件状态。
// ============================================================================
std::vector<hardware_interface::StateInterface> RobotHardwareNode::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // 导出所有关节的状态接口
  for (std::size_t joint_index = 0; joint_index < info_.joints.size(); ++joint_index) 
  {
    for (std::size_t interface_index = 0;
      interface_index < info_.joints[joint_index].state_interfaces.size();
      ++interface_index)
    {
      state_interfaces.emplace_back(
        info_.joints[joint_index].name,
        info_.joints[joint_index].state_interfaces[interface_index].name,
        &joints_[joint_index].state_values[interface_index]);   // 指向实际存储的指针
    }
  }

  // 导出所有传感器的状态接口
  for (std::size_t sensor_index = 0; sensor_index < info_.sensors.size(); ++sensor_index) {
    for (std::size_t interface_index = 0;
      interface_index < info_.sensors[sensor_index].state_interfaces.size();
      ++interface_index)
    {
      state_interfaces.emplace_back(
        info_.sensors[sensor_index].name,
        info_.sensors[sensor_index].state_interfaces[interface_index].name,
        &sensors_[sensor_index].state_values[interface_index]);
    }
  }

  return state_interfaces;
}

// ============================================================================
// 导出命令接口 (command interfaces)
// 将 joints_ 中 command_values 的地址暴露给控制框架，控制器会向这些地址写入目标值。
// ============================================================================
std::vector<hardware_interface::CommandInterface> RobotHardwareNode::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (std::size_t joint_index = 0; joint_index < info_.joints.size(); ++joint_index) 
  {
    for (std::size_t interface_index = 0;
      interface_index < info_.joints[joint_index].command_interfaces.size();
      ++interface_index)
    {
      command_interfaces.emplace_back(
        info_.joints[joint_index].name,
        info_.joints[joint_index].command_interfaces[interface_index].name,
        &joints_[joint_index].command_values[interface_index]);  // 指向实际存储的指针
    }
  }

  return command_interfaces;
}

// ============================================================================
// 生命周期回调: on_configure
// 在 on_init 之后，从非激活到未激活状态时调用。
// 此时应创建具体的硬件驱动对象 (MotorDriver, IMUDriver)，
// 但尚未使能设备。
// ============================================================================
hardware_interface::CallbackReturn RobotHardwareNode::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
  rclcpp::get_logger("robot_hardware_node"),
  "硬件 驱动层初始化开始 -------------------------------------------------------------");

  try {
    // 根据关节的参数创建对应的电机驱动对象
    for (auto & joint : joints_) {
      joint.motor = MotorDriver::create_motor(
        joint.motor_id,
        joint.interface_type,
        joint.interface,
        joint.motor_type,
        joint.motor_model,
        joint.master_id_offset,
        joint.zero_offset);
    }

    // 根据传感器参数创建 IMU 驱动对象
    for (auto & sensor : sensors_) {
      sensor.imu = IMUDriver::create_imu(
        sensor.interface_type,
        sensor.interface,
        sensor.imu_type,
        sensor.baudrate);
    }
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      rclcpp::get_logger("robot_hardware_node"),
      "未能成功创建硬件实例: %s",
      exception.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 创建急停 topic 订阅 (独立节点 + 独立 executor + 后台 spin 线程)
  try {
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
    RCLCPP_INFO(
      rclcpp::get_logger("robot_hardware_node"),
      "急停 topic 订阅已创建: ~/estop");
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      rclcpp::get_logger("robot_hardware_node"),
      "急停订阅创建失败: %s",
      exception.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
  rclcpp::get_logger("robot_hardware_node"),
  "硬件 驱动层初始化完成 -------------------------------------------------------------");

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// 生命周期回调: on_activate
// 进入激活状态时调用，此时应使能硬件 (例如上电、使能电机)，
// 并将电机设置为 MIT 控制模式。
// ============================================================================
hardware_interface::CallbackReturn RobotHardwareNode::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try {
    for (auto & joint : joints_) {
      joint.motor->init_motor();                         // 电机初始化(可能包括CAN通信配置等)
      joint.motor->set_motor_control_mode(MotorDriver::MIT); // 设置为MIT模式(力矩控制)
      joint.active = true;                               // 标记为已激活
    }
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      rclcpp::get_logger("robot_hardware_node"),
      "使能电机失败 motors: %s",
      exception.what());
    return hardware_interface::CallbackReturn::ERROR;
  }


  RCLCPP_INFO(rclcpp::get_logger("robot_hardware_node"), "电机硬件使能 -------------------------------------------------------------");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// 生命周期回调: on_deactivate
// 退出激活状态时调用，应安全地停止所有电机并释放资源。
// 这里发送零力矩命令，然后调用 deinit_motor 关闭使能。
// ============================================================================
hardware_interface::CallbackReturn RobotHardwareNode::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto & joint : joints_) {
    if (!joint.motor || !joint.active) {
      continue;
    }

    try {
      // 发送零力矩命令，使电机无力矩输出，防止掉使能时突然运动
      joint.motor->motor_mit_cmd(0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
      joint.motor->deinit_motor();   // 去使能，关闭通信或电源

    } catch (const std::exception & exception) {
      RCLCPP_WARN(
        rclcpp::get_logger("robot_hardware_node"),
        "未能安全的失能  motor %u: %s",
        joint.motor_id,
        exception.what());
    }
    joint.active = false;           // 清除激活标志
  }

  // 停止急停订阅和 spin 线程
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

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// 实时循环: read()
// 从硬件读取所有关节和传感器的当前状态，并填入 state_values 中。
// 支持多总线并行读取，利用线程池提高效率。
// ============================================================================
hardware_interface::return_type RobotHardwareNode::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  try {
    // 并行遍历每条总线，读取该总线上所有电机的状态
    for_each_bus_parallel([this](const BusData & bus) {
      for (const auto joint_index : bus.joint_indices) {
        auto & joint = joints_[joint_index];
        if (!joint.motor) {
          continue;
        }

        // 如果配置了位置状态接口，读取电机位置
        if (joint.has_position_state && joint.position_state_index < joint.state_values.size()) {
          joint.state_values[joint.position_state_index] = joint.motor->get_motor_pos();
        }
        // 如果配置了速度状态接口，读取电机速度
        if (joint.has_velocity_state && joint.velocity_state_index < joint.state_values.size()) {
          joint.state_values[joint.velocity_state_index] = joint.motor->get_motor_spd();
        }
        joint.motor->refresh_motor_status();  // STW: 周期查询0xA3角度
      }
    });

    // 遍历所有传感器，读取 IMU 数据并填充对应的状态字段
    for (std::size_t sensor_index = 0; sensor_index < sensors_.size(); ++sensor_index) {
      auto & sensor = sensors_[sensor_index];
      if (!sensor.imu) {
        continue;
      }

      const auto quat = sensor.imu->get_quat();       // 获取四元数 [w, x, y, z]
      const auto ang_vel = sensor.imu->get_ang_vel(); // 角速度 [x, y, z]
      const auto lin_acc = sensor.imu->get_lin_acc(); // 线性加速度 [x, y, z]

      if (quat.size() >= 4) {
        if (sensor.has_orientation_w && sensor.orientation_w_index < sensor.state_values.size()) {
          sensor.state_values[sensor.orientation_w_index] = quat[0];
        }
        if (sensor.has_orientation_x && sensor.orientation_x_index < sensor.state_values.size()) {
          sensor.state_values[sensor.orientation_x_index] = quat[1];
        }
        if (sensor.has_orientation_y && sensor.orientation_y_index < sensor.state_values.size()) {
          sensor.state_values[sensor.orientation_y_index] = quat[2];
        }
        if (sensor.has_orientation_z && sensor.orientation_z_index < sensor.state_values.size()) {
          sensor.state_values[sensor.orientation_z_index] = quat[3];
        }
      }

      if (ang_vel.size() >= 3) {
        if (
          sensor.has_angular_velocity_x &&
          sensor.angular_velocity_x_index < sensor.state_values.size())
        {
          sensor.state_values[sensor.angular_velocity_x_index] = ang_vel[0];
        }
        if (
          sensor.has_angular_velocity_y &&
          sensor.angular_velocity_y_index < sensor.state_values.size())
        {
          sensor.state_values[sensor.angular_velocity_y_index] = ang_vel[1];
        }
        if (
          sensor.has_angular_velocity_z &&
          sensor.angular_velocity_z_index < sensor.state_values.size())
        {
          sensor.state_values[sensor.angular_velocity_z_index] = ang_vel[2];
        }
      }

      if (lin_acc.size() >= 3) {
        if (
          sensor.has_linear_acceleration_x &&
          sensor.linear_acceleration_x_index < sensor.state_values.size())
        {
          sensor.state_values[sensor.linear_acceleration_x_index] = lin_acc[0];
        }
        if (
          sensor.has_linear_acceleration_y &&
          sensor.linear_acceleration_y_index < sensor.state_values.size())
        {
          sensor.state_values[sensor.linear_acceleration_y_index] = lin_acc[1];
        }
        if (
          sensor.has_linear_acceleration_z &&
          sensor.linear_acceleration_z_index < sensor.state_values.size())
        {
          sensor.state_values[sensor.linear_acceleration_z_index] = lin_acc[2];
        }
      }
    }
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      rclcpp::get_logger("robot_hardware_node"),
      "读取多总线电机状态失败: %s",
      exception.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

// ============================================================================
// 实时循环: write()
// 从命令接口 (command_values) 中读取控制器下发的指令，
// 发送到对应电机。此处仅使用力矩命令 (effort)，且电机工作在MIT模式。
// ============================================================================
hardware_interface::return_type RobotHardwareNode::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  try {
    // 急停标志在主线程读一次，lambda 捕获副本，避免每次循环都 atomic load
    const bool estop_active = estop_flag_.load();

    // 并行处理各总线上的电机命令发送
    for_each_bus_parallel([this, estop_active](const BusData & bus) {
      for (const auto joint_index : bus.joint_indices) {
        auto & joint = joints_[joint_index];
        if (!joint.motor || !joint.active) {
          continue;
        }

        // 急停优先：使用电机内部 kd 阻尼，替换控制器力矩命令
        if (estop_active) {
          joint.motor->estop(joint.estop_kd);
          continue;
        }

        double effort_command = 0.0;
        // 如果有力矩命令接口，读取其值；若非有限(如NaN)，置0
        if (joint.has_effort_command && joint.effort_command_index < joint.command_values.size()) {
          const double command_value = joint.command_values[joint.effort_command_index];
          effort_command = std::isfinite(command_value) ? command_value : 0.0;
        }

        // 根据方向系数调整力矩方向
        const double torque_command = effort_command * joint.direction;

        // 将力矩限制在 [-1.0, 1.0] 范围内 (假设力矩归一化)
        //const double torque = std::clamp(torque_command, -1.0, 1.0);

        // MIT 模式命令: 期望位置0, 期望速度0, 前馈力矩为计算出的torque
        joint.motor->motor_mit_cmd(
          0.0F,
          0.0F,
          0.0F,
          0.0F,
          static_cast<float>(torque_command));
      }
    });
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      rclcpp::get_logger("robot_hardware_node"),
      "写入多总线电机命令失败: %s",
      exception.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

// ============================================================================
// 辅助函数: 参数读取
// 从关节或传感器的 <param> 标签中获取指定的参数值。
// get_required_* 在参数缺失时抛出异常，保证必须配置。
// ============================================================================
std::string RobotHardwareNode::get_joint_param(
  const hardware_interface::ComponentInfo & joint_info,
  const std::string & name,
  const std::string & default_value) const
{
  const auto iter = joint_info.parameters.find(name);
  return iter == joint_info.parameters.end() ? default_value : iter->second;
}

std::string RobotHardwareNode::get_required_joint_param(
  const hardware_interface::ComponentInfo & joint_info,
  const std::string & name) const
{
  const auto iter = joint_info.parameters.find(name);
  if (iter == joint_info.parameters.end()) {
    throw std::runtime_error("Missing required parameter '" + name + "' for '" + joint_info.name + "'");
  }
  return iter->second;
}

int RobotHardwareNode::get_joint_int_param(
  const hardware_interface::ComponentInfo & joint_info,
  const std::string & name,
  std::size_t default_value) const
{
  const auto iter = joint_info.parameters.find(name);
  return iter == joint_info.parameters.end() ? static_cast<int>(default_value) : std::stoi(iter->second);
}

int RobotHardwareNode::get_required_joint_int_param(
  const hardware_interface::ComponentInfo & joint_info,
  const std::string & name) const
{
  const auto value = get_required_joint_param(joint_info, name);
  try {
    return std::stoi(value);
  } catch (const std::exception &) {
    throw std::runtime_error(
      "Invalid integer parameter '" + name + "' for '" + joint_info.name + "': " + value);
  }
}

double RobotHardwareNode::get_joint_double_param(
  const hardware_interface::ComponentInfo & joint_info,
  const std::string & name,
  double default_value) const
{
  const auto iter = joint_info.parameters.find(name);
  return iter == joint_info.parameters.end() ? default_value : std::stod(iter->second);
}

double RobotHardwareNode::get_required_joint_double_param(
  const hardware_interface::ComponentInfo & joint_info,
  const std::string & name) const
{
  const auto value = get_required_joint_param(joint_info, name);
  try {
    return std::stod(value);
  } catch (const std::exception &) {
    throw std::runtime_error(
      "Invalid double parameter '" + name + "' for '" + joint_info.name + "': " + value);
  }
}

// ============================================================================
// 便捷接口: 手动设置状态和读取命令 (可用于调试或更复杂的逻辑)
// ============================================================================
void RobotHardwareNode::set_joint_state(
  std::size_t joint_index,
  const std::string & interface_name,
  double value)
{
  const auto & interfaces = info_.joints[joint_index].state_interfaces;
  const auto iter = std::find_if(
    interfaces.begin(),
    interfaces.end(),
    [&interface_name](const auto & interface) {
      return interface.name == interface_name;
    });
  if (iter == interfaces.end()) {
    return;
  }
  const auto interface_index = static_cast<std::size_t>(std::distance(interfaces.begin(), iter));
  joints_[joint_index].state_values[interface_index] = value;
}

void RobotHardwareNode::set_sensor_state(
  std::size_t sensor_index,
  const std::string & interface_name,
  double value)
{
  const auto & interfaces = info_.sensors[sensor_index].state_interfaces;
  const auto iter = std::find_if(
    interfaces.begin(),
    interfaces.end(),
    [&interface_name](const auto & interface) {
      return interface.name == interface_name;
    });
  if (iter == interfaces.end()) {
    return;
  }
  const auto interface_index = static_cast<std::size_t>(std::distance(interfaces.begin(), iter));
  sensors_[sensor_index].state_values[interface_index] = value;
}

double RobotHardwareNode::get_joint_command(
  std::size_t joint_index,
  const std::string & interface_name,
  double default_value) const
{
  const auto & interfaces = info_.joints[joint_index].command_interfaces;
  const auto iter = std::find_if(
    interfaces.begin(),
    interfaces.end(),
    [&interface_name](const auto & interface) {
      return interface.name == interface_name;
    });
  if (iter == interfaces.end()) {
    return default_value;
  }
  const auto interface_index = static_cast<std::size_t>(std::distance(interfaces.begin(), iter));
  const double value = joints_[joint_index].command_values[interface_index];
  return std::isfinite(value) ? value : default_value;
}

// ============================================================================
// 总线分组与并行执行
// ============================================================================

// 根据关节的 interface 字段重新构建 CAN 总线分组。
// 每个不同的 interface 字符串代表一条独立的物理总线。
void RobotHardwareNode::rebuild_bus_groups()
{
  buses_.clear();
  std::unordered_map<std::string, std::size_t> bus_index_by_interface;

  for (std::size_t joint_index = 0; joint_index < joints_.size(); ++joint_index) {
    const auto & interface = joints_[joint_index].interface;
    auto iter = bus_index_by_interface.find(interface);
    if (iter == bus_index_by_interface.end()) {
      // 新总线，创建新的 BusData 并记录映射
      const std::size_t bus_index = buses_.size();
      bus_index_by_interface.emplace(interface, bus_index);
      buses_.push_back(BusData{interface, {joint_index}});
      continue;
    }
    // 已有总线，将关节索引加入对应 bus 的 joint_indices 列表
    buses_[iter->second].joint_indices.push_back(joint_index);
  }

  // 打印每条总线包含的关节数量
  for (const auto & bus : buses_) {
    RCLCPP_INFO(
      rclcpp::get_logger("robot_hardware_node"),
      "CAN bus %s has %zu joints.",
      bus.interface.c_str(),
      bus.joint_indices.size());
  }
}

// 对每条总线执行给定的 task。如果有多条总线且线程池存在，则并行执行；
// 否则单线程顺序执行。
void RobotHardwareNode::for_each_bus_parallel(const std::function<void(const BusData &)> & task)
{
  if (buses_.empty()) {
    return;
  }

  // 仅有一条总线或线程池未创建时，直接顺序调用
  if (buses_.size() == 1 || !bus_thread_pool_) {
    for (const auto & bus : buses_) {
      task(bus);
    }
    return;
  }

  // 多总线并行：创建与总线数量相等的任务，丢给线程池运行
  bus_tasks_.clear();
  if (bus_tasks_.capacity() < buses_.size()) {
    bus_tasks_.reserve(buses_.size());
  }
  for (const auto & bus : buses_) {
    const auto * bus_ptr = &bus;
    bus_tasks_.emplace_back([&task, bus_ptr]() {
      task(*bus_ptr);
    });
  }
  bus_thread_pool_->run_parallel(bus_tasks_);
}

}  // namespace robot_hardware

// 插件导出宏，将 RobotHardwareNode 注册为 ros2_control 的硬件接口插件
PLUGINLIB_EXPORT_CLASS(robot_hardware::RobotHardwareNode, hardware_interface::SystemInterface)