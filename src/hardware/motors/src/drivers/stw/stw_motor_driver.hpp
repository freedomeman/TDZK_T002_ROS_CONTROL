#pragma once

#include <atomic>
#include <memory>
#include <string>

#include "motor_driver.hpp"
#include "protocol/can/socket_can.hpp"

// ============================================================
// STW (GDZ / ZE300 series) Motor Driver
//
// Protocol: GDZ 自定义 CAN 通信协议 V3.07
// - CAN standard frame, 1Mbps default
// - Dev_addr default = 0x01
// - MIT 运控模式: CAN ID = 0x400 | Dev_addr
// - 控制命令: CAN ID = Dev_addr / (0x100|Dev_addr) / 0x00 / 0xFF
// - 应答: CAN ID = Dev_addr
// ============================================================

enum class StwMotorModel : int {
  STW_GIM_DEFAULT = 0,
  NumOfMotor
};

struct StwLimitParam {
  float PosMax;   // rad
  float SpdMax;   // rad/s
  float TauMax;   // N·m
  float OKpMax;   // 500
  float OKdMax;   // 5
};

// GDZ 协议命令码
namespace GdzCmd {
  constexpr uint8_t RESTART      = 0x00;
  constexpr uint8_t READ_VER     = 0xA0;
  constexpr uint8_t READ_CURRENT = 0xA1;
  constexpr uint8_t READ_SPEED   = 0xA2;
  constexpr uint8_t READ_ANGLE   = 0xA3;
  constexpr uint8_t READ_STATUS  = 0xAE;
  constexpr uint8_t CLEAR_FAULT  = 0xAF;
  constexpr uint8_t SET_ORIGIN   = 0xB1;
  constexpr uint8_t SET_POS_SPD  = 0xB2;
  constexpr uint8_t SET_MAX_CUR  = 0xB3;
  constexpr uint8_t Q_CUR_CTRL   = 0xC0;   // 力矩控制
  constexpr uint8_t SPD_CTRL     = 0xC1;   // 速度控制
  constexpr uint8_t POS_CTRL     = 0xC2;   // 绝对值位置
  constexpr uint8_t REL_POS_CTRL = 0xC3;   // 相对位置
  constexpr uint8_t BRAKE_CTRL   = 0xCE;   // 抱闸
  constexpr uint8_t MOTOR_FREE   = 0xCF;   // 关闭电机（自由态）
  constexpr uint8_t CONFIG_MT    = 0xF0;   // 运控参数读写
  constexpr uint8_t READ_MT      = 0xF1;   // 读运控实时数据
  // MIT 运控帧: BIT_ID = 0x400 | Dev_addr, 无命令码
  constexpr uint16_t MIT_ID_BASE = 0x400;
}

class StwMotorDriver : public MotorDriver {
 public:
  StwMotorDriver(uint16_t motor_id,
                 const std::string& interface_type,
                 const std::string& can_interface,
                 uint16_t master_id_offset,
                 StwMotorModel motor_model,
                 double motor_zero_offset = 0.0);
  ~StwMotorDriver();

  void lock_motor() override;
  void unlock_motor() override;
  uint8_t init_motor() override;
  void deinit_motor() override;
  bool set_motor_zero() override;
  bool write_motor_flash() override;

  void get_motor_param(uint8_t param_cmd) override;
  void motor_pos_cmd(float pos, float spd, bool ignore_limit) override;
  void motor_spd_cmd(float spd) override;
  void reset_motor_id() override {}
  void motor_mit_cmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) override;
  void set_motor_control_mode(uint8_t motor_control_mode) override;
  int get_response_count() const override { return response_count_; }
  void refresh_motor_status() override;
  void clear_motor_error() override;

  // 摩擦补偿参数设置 (单位: N·m / N·m·s/rad / rad/s)
  void set_friction_params(float tc, float bv, float epsilon) {
    friction_tc_ = tc;
    friction_bv_ = bv;
    friction_epsilon_ = epsilon;
  }

 private:
  void can_rx_cbk(const can_frame& rx_frame);
  void parse_0xf1_response(const can_frame& rx_frame);
  float friction_compensation(float torque_des_nm);

  uint16_t mit_can_id() const { return GdzCmd::MIT_ID_BASE | motor_id_; }

  uint16_t master_id_{0};
  std::atomic<int> response_count_{0};
  StwMotorModel motor_model_{StwMotorModel::STW_GIM_DEFAULT};
  StwLimitParam limit_param_{};
  std::string can_interface_;
  std::shared_ptr<MotorsSocketCAN> can_;

  // 摩擦补偿参数 (物理单位, 默认值来自标定, 可运行时调用 setter 更新)
  float friction_tc_{0.0033f};      // 库仑/静摩擦 N·m
  float friction_bv_{0.0031f};      // 粘性阻尼 N·m·s/rad
  float friction_epsilon_{0.01f};   // 速度死区 rad/s
};
