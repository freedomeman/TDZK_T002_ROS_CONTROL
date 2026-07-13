#include "t002_controller/t002_controller.hpp"
#include "t002_controller/user_code.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"

namespace t002_controller
{

controller_interface::CallbackReturn T002Controller::on_init() //初始化函数
{
  RCLCPP_INFO(get_node()->get_logger(), "T002Controller on_init ---");
  if (!get_node()->get_parameter("joints", joint_names_)) {//如果关节参数是空的加返回，并且输出ERROR。并且把关节参数写入joint_names_
    RCLCPP_ERROR(get_node()->get_logger(), "Missing 'joints' parameter");
    return controller_interface::CallbackReturn::ERROR;
  }
  try {
    get_node()->get_parameter("sensors", sensor_names_);//使用try和catch，获取传感器数据，如果出错就清空
  } catch (const rclcpp::exceptions::InvalidParameterValueException &) {
    sensor_names_.clear();
  }
  command_interface_types_ =
    get_node()->get_parameter("command_interfaces").as_string_array();//获取参数。但这里是什么作用还不清楚
  state_interface_types_ =
    get_node()->get_parameter("state_interfaces").as_string_array();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
T002Controller::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf; //创建配置对象
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL; //配置类型是INDIVIDUAL
  for (const auto & name : joint_names_) //遍历所有关节
    for (const auto & iface : command_interface_types_) //便利所有命令类型
      conf.names.push_back(name + "/" + iface); //按照此格式拼接，并且加入列表
  return conf;
}

controller_interface::InterfaceConfiguration
T002Controller::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;//作用同上
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & name : joint_names_)
    for (const auto & iface : state_interface_types_)
      conf.names.push_back(name + "/" + iface);
  for (const auto & name : sensor_names_) {
    conf.names.push_back(name + "/orientation.x");
    conf.names.push_back(name + "/orientation.y");
    conf.names.push_back(name + "/orientation.z");
    conf.names.push_back(name + "/orientation.w");
    conf.names.push_back(name + "/angular_velocity.x");
    conf.names.push_back(name + "/angular_velocity.y");
    conf.names.push_back(name + "/angular_velocity.z");
  }
  return conf;
}


namespace {
std::vector<double> parse_semicolon_list(const std::vector<std::string> & strs) {
  std::vector<double> out;
  for (const auto & s : strs) {
    std::istringstream iss(s);
    std::string token;
    while (std::getline(iss, token, ';')) {
      try { out.push_back(std::stod(token)); }
      catch (...) { out.push_back(0.0); }
    }
  }
  return out;
}
}  // namespace

controller_interface::CallbackReturn T002Controller::on_configure(
  const rclcpp_lifecycle::State & /*prev*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "T002Controller on_configure ---");

  std::vector<double> pd_flat;
  {
    std::vector<std::string> pd_strs;
    if (get_node()->get_parameter("pd", pd_strs)) {
      pd_flat = parse_semicolon_list(pd_strs);
    } else {
      std::vector<double> pd_dbls;
      if (get_node()->get_parameter("pd", pd_dbls)) {
        pd_flat = pd_dbls;
      }
    }
    if (pd_flat.empty()) {
      RCLCPP_WARN(get_node()->get_logger(), "Missing 'pd', using 0");
      pd_flat.assign(joint_names_.size() * 2U, 0.0);
    }
  }
  pd_kps_.resize(joint_names_.size(), 0.0);//把pd参数变成和joint长度一样
  pd_kds_.resize(joint_names_.size(), 0.0);
  for (std::size_t i = 0U; i < joint_names_.size(); ++i) { //将扁平化存储的 PD 参数（pd_flat）拆解并分别赋值给比例增益容器（pd_kps_）和微分增益容器（pd_kds_）
    std::size_t b = i * 2U;
    if (b + 1U < pd_flat.size()) {
      pd_kps_[i] = pd_flat[b];
      pd_kds_[i] = pd_flat[b + 1U];
    }
  }

  {
    std::vector<std::string> jl_strs;
    if (get_node()->get_parameter("joint_limits", jl_strs))
      joint_limits_ = parse_semicolon_list(jl_strs);
    else
      get_node()->get_parameter("joint_limits", joint_limits_);
  }
  {
    std::vector<std::string> el_strs;
    if (get_node()->get_parameter("effort_limits", el_strs))
      effort_limits_ = parse_semicolon_list(el_strs);
    else
      get_node()->get_parameter("effort_limits", effort_limits_);
  }
  get_node()->get_parameter("default_joint_positions", default_joint_positions_);

  // 读取 joint_modes
  {
    std::vector<std::string> mode_strs;
    if (get_node()->get_parameter("joint_modes", mode_strs)) {
      joint_modes_.reserve(mode_strs.size());
      for (const auto & s : mode_strs) {
        if (s == "position") joint_modes_.push_back(JointMode::Position);
        else if (s == "velocity") joint_modes_.push_back(JointMode::Velocity);
        else if (s == "torque") joint_modes_.push_back(JointMode::Torque);
        else joint_modes_.push_back(JointMode::Position);  // 默认
      }
    }
    if (joint_modes_.size() < joint_names_.size())
      joint_modes_.resize(joint_names_.size(), JointMode::Position);
  }

  joints_.clear(); //清空堆
  joints_.reserve(joint_names_.size()); //把长度改成和joint_name一样的
  for (const auto & name : joint_names_)
    joints_.push_back(std::make_shared<Joint>(Joint{name, 0.0, {}, {}, {}})); //在里面追加元素，遍历的方式
  if (default_joint_positions_.size() < joint_names_.size())
    default_joint_positions_.resize(joint_names_.size(), 0.0);

  RCLCPP_INFO(get_node()->get_logger(), "Configured %zu joints", joint_names_.size());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn T002Controller::on_activate(
  const rclcpp_lifecycle::State & /*prev*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "T002Controller on_activate ---");

  for (std::size_t idx = 0U; idx < joints_.size(); ++idx) {
    const auto & name = joints_[idx]->name;

    // Position state (read)
    for (auto & iface : state_interfaces_)
      if (iface.get_prefix_name() == name &&
          iface.get_interface_name() == hardware_interface::HW_IF_POSITION)
        { joints_[idx]->position_handle = std::ref(iface); break; }

    // Velocity state (read)
    for (auto & iface : state_interfaces_)
      if (iface.get_prefix_name() == name &&
          iface.get_interface_name() == hardware_interface::HW_IF_VELOCITY)
        { joints_[idx]->velocity_handle = std::ref(iface); break; }

    // Effort command (write)
    bool found = false;
    for (auto & iface : command_interfaces_)
      if (iface.get_prefix_name() == name &&
          iface.get_interface_name() == hardware_interface::HW_IF_EFFORT)
        { joints_[idx]->effort_command_handle = std::ref(iface); found = true; break; }
    if (!found) {
      RCLCPP_ERROR(get_node()->get_logger(), "No effort cmd for '%s'", name.c_str());
      return controller_interface::CallbackReturn::FAILURE;
    }

    double dp = (idx < default_joint_positions_.size())
      ? default_joint_positions_[idx] : 0.0;
    joints_[idx]->desired_position = dp;
  }

  cmd_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    command_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr m) { command_callback(m); });

  RCLCPP_INFO(get_node()->get_logger(), "Activated %zu joints", joints_.size());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn T002Controller::on_deactivate(
  const rclcpp_lifecycle::State & /*prev*/)
{
  cmd_sub_.reset();
  release_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type T002Controller::update(
  const rclcpp::Time & /*t*/, const rclcpp::Duration & /*p*/)
{

  for (std::size_t i = 0U; i < joints_.size(); ++i) {
    if (!joints_[i]->effort_command_handle.has_value()) continue;

    const auto mode = (i < joint_modes_.size()) ? joint_modes_[i] : JointMode::Position;
    double effort = 0.0;

    if (mode == JointMode::Torque) {
      // 力矩模式: 直接透传
      effort = joints_[i]->desired_position;
    } else {
      // 读取实际状态
      double actual_pos = normalizeAnglePi((joints_[i]->position_handle.has_value())
        ? joints_[i]->position_handle->get().get_value() : 0.0);
      double actual_vel = (joints_[i]->velocity_handle.has_value())
        ? joints_[i]->velocity_handle->get().get_value() : 0.0;
      double cmd = joints_[i]->desired_position;
      double kp = (i < pd_kps_.size()) ? pd_kps_[i] : 0.0;
      double kd = (i < pd_kds_.size()) ? pd_kds_[i] : 0.0;

      if (mode == JointMode::Velocity) {
        // 速度模式: effort = Kp * (cmd_vel - actual_vel)
        effort = kp * (cmd - actual_vel);
      } else {
        // 位置模式: effort = Kp * (cmd_pos - actual_pos) - Kd * actual_vel
        effort = kp * normalizeAnglePi(cmd - actual_pos) - kd * actual_vel;
      }
    }

    effort = clamp_effort(i, effort);
    // effort = 0.0;
    joints_[i]->effort_command_handle->get().set_value(effort);

  }
  return controller_interface::return_type::OK;
  
}

void T002Controller::command_callback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() != joints_.size()) {
    RCLCPP_WARN(get_node()->get_logger(), "cmd size %zu != %zu",
                msg->data.size(), joints_.size());
    return;
  }
  for (std::size_t i = 0U; i < joints_.size(); ++i) {
    const auto mode = (i < joint_modes_.size()) ? joint_modes_[i] : JointMode::Position;
    if (mode == JointMode::Torque)
      joints_[i]->desired_position = msg->data[i];  // 力矩模式不钳位
    else
      joints_[i]->desired_position = sanitize_position(i, msg->data[i]);
  }
}

double T002Controller::get_state_value(
  const hardware_interface::LoanedStateInterface & iface) const
{ return iface.get_value(); }

double T002Controller::compute_pd_effort(std::size_t i, const Joint & j) const
{
  const double pos = normalizeAnglePi(j.position_handle ? get_state_value(j.position_handle->get()) : 0.0) ;
  const double vel = j.velocity_handle
    ? get_state_value(j.velocity_handle->get()) : 0.0;
  const double kp = i < pd_kps_.size() ? pd_kps_[i] : 0.0;
  const double kd = i < pd_kds_.size() ? pd_kds_[i] : 0.0;

  double e = kp * normalizeAnglePi(j.desired_position - pos) - kd * vel;
  if (!std::isfinite(e)) e = 0.0;
  return clamp_effort(i, e);
}

double T002Controller::sanitize_position(std::size_t i, double v) const
{
  if (!std::isfinite(v)) return 0.0;
  std::size_t lo = i * 2U, hi = lo + 1U;
  if (hi < joint_limits_.size()) {
    v = std::max(v, joint_limits_[lo]);
    v = std::min(v, joint_limits_[hi]);
  }
  return v;
}

double T002Controller::clamp_effort(std::size_t i, double e) const
{
  if (!std::isfinite(e)) e = 0.0;
  std::size_t lo = i * 2U, hi = lo + 1U;
  if (hi < effort_limits_.size()) {
    e = std::max(e, effort_limits_[lo]);
    e = std::min(e, effort_limits_[hi]);
  }
  return e;
}

/**
 * 将角度归一化到 [-π, π] 区间。
 * 使用 atan2(sin, cos) 方法，数值稳定。
 */
double T002Controller::normalizeAnglePi(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace t002_controller

PLUGINLIB_EXPORT_CLASS(
  t002_controller::T002Controller,
  controller_interface::ControllerInterface)
