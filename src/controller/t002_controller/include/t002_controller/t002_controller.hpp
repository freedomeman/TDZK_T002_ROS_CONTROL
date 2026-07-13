#ifndef T002_CONTROLLER__T002_CONTROLLER_HPP_
#define T002_CONTROLLER__T002_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/subscription.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace t002_controller
{

enum class JointMode { Position, Velocity, Torque };

struct Joint
{
  std::string name;
  double desired_position{0.0};
  // state (read)
  std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    position_handle;
  std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    velocity_handle;
  // command (write)
  std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    effort_command_handle;
};

class T002Controller : public controller_interface::ControllerInterface
{
public:
  T002Controller() = default;
  ~T002Controller() override = default;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  static double normalizeAnglePi(double angle);

private:
  void command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  double get_state_value(
    const hardware_interface::LoanedStateInterface & iface) const;
  double compute_pd_effort(std::size_t idx, const Joint & joint) const;
  double sanitize_position(std::size_t idx, double value) const;
  double clamp_effort(std::size_t idx, double effort) const;


  std::vector<std::string> joint_names_;
  std::vector<std::string> sensor_names_;
  std::vector<std::string> command_interface_types_;
  std::vector<std::string> state_interface_types_;

  std::vector<double> default_joint_positions_;
  std::vector<double> joint_limits_;
  std::vector<double> pd_kps_;
  std::vector<double> pd_kds_;
  std::vector<double> effort_limits_;
  std::vector<JointMode> joint_modes_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;
  std::string command_topic_{"~/command"};

  std::vector<std::shared_ptr<Joint>> joints_;
};

}  // namespace t002_controller
#endif
