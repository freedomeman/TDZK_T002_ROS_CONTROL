#include <cmath>
#include <cstring>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <fcntl.h>
#include <linux/joystick.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class XboxText : public rclcpp::Node
{
public:
  XboxText()
  : Node("steer_xboxcontrol")
  {
    this->declare_parameter("device", "/dev/input/js0");
    this->declare_parameter("publish_rate", 50.0);

    const std::string device_path = this->get_parameter("device").as_string();
    const double publish_rate = this->get_parameter("publish_rate").as_double();

    js_fd_ = open(device_path.c_str(), O_RDONLY | O_NONBLOCK);
    if (js_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(),
        "Cannot open joystick device '%s': %s",
        device_path.c_str(), std::strerror(errno));
      return;
    }

    uint8_t axis_count = 0;
    uint8_t button_count = 0;
    if (ioctl(js_fd_, JSIOCGAXES, &axis_count) < 0) { axis_count = 8; }
    if (ioctl(js_fd_, JSIOCGBUTTONS, &button_count) < 0) { button_count = 16; }

    RCLCPP_INFO(this->get_logger(),
      "Opened '%s' (%d axes, %d buttons)", device_path.c_str(), axis_count, button_count);

    axes_.resize(axis_count, 0);
    buttons_.resize(button_count, 0);

    pub_pose_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/target_pose", 10);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate);
    timer_ = this->create_wall_timer(period, std::bind(&XboxText::timer_callback, this));
    last_time_ = this->get_clock()->now();

    RCLCPP_INFO(this->get_logger(), "XboxText ready → /target_pose (积分模式)");
    RCLCPP_INFO(this->get_logger(),
      "  ch2→yaw_rate[%.3f]  ch5→pitch_rate[%.3f]  ch4→roll_rate[%.3f]",
      0.3, 0.3, 0.3);
  }

  ~XboxText() override
  {
    if (js_fd_ >= 0) { close(js_fd_); }
  }

private:
  void timer_callback()
  {
    if (js_fd_ < 0) { return; }

    struct js_event ev;
    ssize_t n;
    while ((n = read(js_fd_, &ev, sizeof(ev))) == sizeof(ev)) {
      if (ev.type & JS_EVENT_INIT) { continue; }
      switch (ev.type & ~JS_EVENT_INIT) {
        case JS_EVENT_AXIS:
          if (ev.number < axes_.size()) { axes_[ev.number] = ev.value; }
          break;
        case JS_EVENT_BUTTON:
          if (ev.number < buttons_.size()) { buttons_[ev.number] = ev.value; }
          break;
      }
    }

    // ── 控制逻辑 ──
    auto now = this->get_clock()->now();
    double dt = (now - last_time_).seconds();
    if (dt < 0.0 || dt > 0.1) { dt = 0.02; }  // 防止异常跳变

    // 脖子 pitch/roll: 左摇杆 积分 (ch5→pitch, ch4→roll)
    pitch_target_ += axis_raw(5) * 0.3 * dt;
    roll_target_  += axis_raw(4) * 0.3 * dt;
    pitch_target_  = std::clamp(pitch_target_, -0.35, 0.35);
    roll_target_   = std::clamp(roll_target_,  -0.35, 0.35);

    // 脖子 yaw: 按键 0(左) / 2(右)
    if (button_pressed(0)) neck_yaw_target_ -= 0.3 * dt;
    if (button_pressed(2)) neck_yaw_target_ += 0.3 * dt;
    neck_yaw_target_ = std::clamp(neck_yaw_target_, -3.14, 3.14);

    // 腰部: 按键 6(右) / 7(左)
    if (button_pressed(6)) waist_target_ -= 0.3 * dt;
    if (button_pressed(7)) waist_target_ += 0.3 * dt;

    // 底盘: 右摇杆 (ch3→vx, ch2→vy), ch0→wz
    vx_target_ = axis_raw(3) * 10.0;
    vy_target_ = axis_raw(2) * 10.0;
    wz_target_ = axis_raw(0) * 24.0;

    last_time_ = now;

    publish_pose();
    log_all_channels();
  }

  inline bool button_pressed(int idx) const {
    return (idx >= 0 && static_cast<size_t>(idx) < buttons_.size()) && buttons_[idx] != 0;
  }

  double axis_raw(int index) const
  {
    if (index < 0 || static_cast<size_t>(index) >= axes_.size()) { return 0.0; }
    return std::clamp(static_cast<double>(axes_[index]) / 32767.0, -1.0, 1.0);
  }

  void publish_pose()
  {
    auto msg = std_msgs::msg::Float64MultiArray();
    // 顺序: [waist, neck_yaw, neck_pitch, neck_roll, vx, vy, wz, reserve]
    msg.data = std::vector<double>{
      waist_target_,
      neck_yaw_target_,
      pitch_target_,
      roll_target_,
      vx_target_,
      vy_target_,
      wz_target_,
      0.0
    };
    pub_pose_->publish(msg);
  }

  void log_all_channels()
  {
    const auto now = this->get_clock()->now();
    static rclcpp::Time last_log = now;
    if ((now - last_log).seconds() < 1.0) { return; }
    last_log = now;

    RCLCPP_INFO(this->get_logger(),
      "waist=%.3f n_yaw=%.3f n_pitch=%.3f n_roll=%.3f "
      "vx=%.2f vy=%.2f wz=%.2f",
      waist_target_, neck_yaw_target_, pitch_target_, roll_target_,
      vx_target_, vy_target_, wz_target_);
  }

  // ── 状态 ──
  double waist_target_{0.0};
  double neck_yaw_target_{0.0};
  double pitch_target_{0.0};
  double roll_target_{0.0};
  double vx_target_{0.0};
  double vy_target_{0.0};
  double wz_target_{0.0};
  rclcpp::Time last_time_;

  // ── 设备 ──────────────────────────────────────────────
  int js_fd_{-1};
  std::vector<int16_t> axes_;
  std::vector<int16_t> buttons_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_pose_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<XboxText>());
  rclcpp::shutdown();
  return 0;
}



