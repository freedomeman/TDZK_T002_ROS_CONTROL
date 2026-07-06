#pragma once

#include <cstdint>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "imu_driver.hpp"
#include "motor_driver.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "thread_pool.hpp"

namespace robot_hardware
{

class RobotHardwareNode : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & hardware_info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:


  struct JointData
  {
    std::string name;
    std::string interface;
    std::string interface_type;
    std::string motor_type;
    int motor_model{0};
    uint16_t master_id_offset{1};
    uint16_t motor_id{0};
    double zero_offset{0.0};
    double direction{1.0};
    bool active{false};
    std::shared_ptr<MotorDriver> motor;
    std::vector<double> state_values;
    std::vector<double> command_values;
    std::size_t position_state_index{0U};
    std::size_t velocity_state_index{0U};
    std::size_t effort_command_index{0U};
    bool has_position_state{false};
    bool has_velocity_state{false};
    bool has_effort_command{false};
  };

  struct BusData
  {
    std::string interface;
    std::vector<std::size_t> joint_indices;
  };

  struct SensorData
  {
    std::string name;
    std::string interface;
    std::string interface_type;
    std::string imu_type;
    int baudrate{0};
    std::shared_ptr<IMUDriver> imu;
    std::vector<double> state_values;
    std::size_t orientation_w_index{0U};
    std::size_t orientation_x_index{0U};
    std::size_t orientation_y_index{0U};
    std::size_t orientation_z_index{0U};
    std::size_t angular_velocity_x_index{0U};
    std::size_t angular_velocity_y_index{0U};
    std::size_t angular_velocity_z_index{0U};
    std::size_t linear_acceleration_x_index{0U};
    std::size_t linear_acceleration_y_index{0U};
    std::size_t linear_acceleration_z_index{0U};
    bool has_orientation_w{false};
    bool has_orientation_x{false};
    bool has_orientation_y{false};
    bool has_orientation_z{false};
    bool has_angular_velocity_x{false};
    bool has_angular_velocity_y{false};
    bool has_angular_velocity_z{false};
    bool has_linear_acceleration_x{false};
    bool has_linear_acceleration_y{false};
    bool has_linear_acceleration_z{false};
  };

  std::string get_joint_param(
    const hardware_interface::ComponentInfo & joint_info,
    const std::string & name,
    const std::string & default_value) const;

  std::string get_required_joint_param(
    const hardware_interface::ComponentInfo & joint_info,
    const std::string & name) const;

  int get_joint_int_param(
    const hardware_interface::ComponentInfo & joint_info,
    const std::string & name,
    std::size_t default_value) const;

  int get_required_joint_int_param(
    const hardware_interface::ComponentInfo & joint_info,
    const std::string & name) const;

  double get_joint_double_param(
    const hardware_interface::ComponentInfo & joint_info,
    const std::string & name,
    double default_value) const;

  double get_required_joint_double_param(
    const hardware_interface::ComponentInfo & joint_info,
    const std::string & name) const;

  void set_joint_state(
    std::size_t joint_index,
    const std::string & interface_name,
    double value);

  void set_sensor_state(
    std::size_t sensor_index,
    const std::string & interface_name,
    double value);

  double get_joint_command(
    std::size_t joint_index,
    const std::string & interface_name,
    double default_value) const;

  void rebuild_bus_groups();
  void for_each_bus_parallel(const std::function<void(const BusData &)> & task);

  std::vector<JointData> joints_;
  std::vector<BusData> buses_;
  std::unique_ptr<ThreadPool> bus_thread_pool_;
  std::vector<std::function<void()>> bus_tasks_;
  std::vector<SensorData> sensors_;
};

}  // namespace robot_hardware
