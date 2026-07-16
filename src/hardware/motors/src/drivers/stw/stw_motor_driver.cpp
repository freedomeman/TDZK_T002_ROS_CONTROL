#include "stw_motor_driver.hpp"

#include <cmath>
#include <stdexcept>

// ============================================================
// Per-model limits
// ============================================================
static const StwLimitParam kStwLimitParams[] = {
  // GDZ motor: Pos_Max≈95.5 (官方Demo默认), Vel_Max/demo=45, T_Max=28
  {95.5F, 45.0F, 28.0F, 500.0F, 5.0F},
};

// ============================================================
// Constructor / Destructor
// ============================================================
StwMotorDriver::StwMotorDriver(uint16_t motor_id,
                               const std::string& interface_type,
                               const std::string& can_interface,
                               uint16_t master_id_offset,
                               StwMotorModel motor_model,
                               double motor_zero_offset)
    : MotorDriver(),
      can_(MotorsSocketCAN::get(can_interface)),
      motor_model_(motor_model)
{
  if (interface_type != "can") {
    throw std::runtime_error("STW driver only supports CAN interface");
  }

  motor_id_ = motor_id;
  master_id_ = motor_id_ + master_id_offset;
  motor_zero_offset_ = motor_zero_offset;
  can_interface_ = can_interface;

  int idx = static_cast<int>(motor_model);
  if (idx >= 0 && idx < static_cast<int>(StwMotorModel::NumOfMotor))
    limit_param_ = kStwLimitParams[idx];

  // 注册回调：监听 Dev_addr (motor_id_) 上的应答帧
  CanCbkFunc cb = std::bind(&StwMotorDriver::can_rx_cbk, this, std::placeholders::_1);
  can_->add_can_callback(cb, motor_id_);
}

StwMotorDriver::~StwMotorDriver() {
  can_->remove_can_callback(motor_id_);
}

// ============================================================
// Lock / Unlock — GDZ 没有使能/失能命令，用 MIT 帧启动，0xCF 关闭
// ============================================================
void StwMotorDriver::lock_motor() {
  motor_mit_cmd(0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
}

void StwMotorDriver::unlock_motor() {
  can_frame tx_frame{};
  tx_frame.can_id  = motor_id_;
  tx_frame.can_dlc = 0x01;
  tx_frame.data[0] = GdzCmd::MOTOR_FREE;
  can_->transmit(tx_frame);
  response_count_++;
}

// ============================================================
// Initialization — 自动归零保证 0xF1 位置有效
// ============================================================
uint8_t StwMotorDriver::init_motor() {
  unlock_motor();
  Timer::sleep_for(normal_sleep_time);

  // 发送零力矩 MIT 帧激活运控模式
  motor_mit_cmd(0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
  Timer::sleep_for(setup_sleep_time);

  // // ★ 自动归零：设当前位置为原点，使 0xF1 位置字段生效
  // set_motor_zero();
  // Timer::sleep_for(setup_sleep_time);

  // 再发一帧确认运控模式
  motor_mit_cmd(0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
  Timer::sleep_for(normal_sleep_time);

  refresh_motor_status();
  Timer::sleep_for(normal_sleep_time);

  if (error_id_ != 0) {
    logger_->warn("STW motor {} fault 0x{:02X}, attempting clear",
                  motor_id_, static_cast<unsigned>(error_id_));
    clear_motor_error();
  }

  return error_id_;
}

void StwMotorDriver::deinit_motor() {
  unlock_motor();
  Timer::sleep_for(normal_sleep_time);
}

// ============================================================
// MIT 运控命令 (CAN ID = 0x400 | motor_id)
// ============================================================
void StwMotorDriver::motor_mit_cmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) {
  f_p *= limit_param_.PosMax;
  f_v *= limit_param_.SpdMax;
  f_t *= limit_param_.TauMax;
  f_t = -f_t;  // STW 左手坐标系 → 右手坐标系

  // 摩擦补偿 (在 N·m 空间, 补偿后 clamp)
  f_t += friction_compensation(f_t);

  //电机控制的是Q轴电流所以就把力矩转换为电流 力矩/减速比/转矩常数
  f_t = f_t/8/0.41;

  f_p = std::clamp(f_p, -limit_param_.PosMax, limit_param_.PosMax);
  f_v = std::clamp(f_v, -limit_param_.SpdMax, limit_param_.SpdMax);
  f_kp = std::clamp(f_kp, 0.0F, limit_param_.OKpMax);
  f_kd = std::clamp(f_kd, 0.0F, limit_param_.OKdMax);
  f_t = std::clamp(f_t, -limit_param_.TauMax, limit_param_.TauMax);

  auto map_u16 = [](float v, float min, float max) -> uint16_t {
    float ratio = (v - min) / (max - min);
    return static_cast<uint16_t>(ratio * 65535.0F);
  };
  auto map_u12 = [](float v, float min, float max) -> uint16_t {
    float ratio = (v - min) / (max - min);
    return static_cast<uint16_t>(ratio * 4095.0F);
  };

  uint16_t p  = map_u16(f_p, -limit_param_.PosMax, limit_param_.PosMax);
  uint16_t v  = map_u12(f_v, -limit_param_.SpdMax, limit_param_.SpdMax);
  uint16_t kp = map_u12(f_kp, 0.0F, limit_param_.OKpMax);
  uint16_t kd = map_u12(f_kd, 0.0F, limit_param_.OKdMax);
  uint16_t t  = map_u12(f_t, -limit_param_.TauMax, limit_param_.TauMax);

  can_frame tx_frame{};
  tx_frame.can_id  = mit_can_id();
  tx_frame.can_dlc = 0x08;
  tx_frame.data[0] = (p >> 8) & 0xFF;
  tx_frame.data[1] = p & 0xFF;
  tx_frame.data[2] = (v >> 4) & 0xFF;
  tx_frame.data[3] = (v & 0x0F) << 4 | ((kp >> 8) & 0x0F);
  tx_frame.data[4] = kp & 0xFF;
  tx_frame.data[5] = (kd >> 4) & 0xFF;
  tx_frame.data[6] = (kd & 0x0F) << 4 | ((t >> 8) & 0x0F);
  tx_frame.data[7] = t & 0xFF;

  can_->transmit(tx_frame);
  response_count_++;
}

void StwMotorDriver::estop(float kd) {
  motor_mit_cmd(0.0f, 0.0f, 0.0f, kd, 0.0f);
}

// 摩擦补偿: 根据期望力矩方向和当前转速, 返回补偿力矩 (N·m)
// 静止: T_comp = Tc * sign(T_des)
// 运动: T_comp = Tc * sign(w) + Bv * w
float StwMotorDriver::friction_compensation(float torque_des_nm) {
  if (friction_tc_ == 0.0f && friction_bv_ == 0.0f) return 0.0f;

  float velocity = get_motor_spd();
  float T_comp;
  if (std::abs(velocity) < friction_epsilon_) {
    T_comp = friction_tc_ * (torque_des_nm > 0.0f ? 1.0f : -1.0f);
  } else {
    T_comp = friction_tc_ * (velocity > 0.0f ? 1.0f : -1.0f)
           + friction_bv_ * velocity;
  }
  return T_comp;
}

// ============================================================
// 位置控制 (0xC2)
// ============================================================
void StwMotorDriver::motor_pos_cmd(float pos, float spd, bool /*ignore_limit*/) {
  pos = std::clamp(pos, -limit_param_.PosMax, limit_param_.PosMax);
  spd = std::clamp(spd, 0.0F, limit_param_.SpdMax);

  can_frame tx_frame{};
  tx_frame.can_id  = motor_id_;
  tx_frame.can_dlc = 0x08;
  tx_frame.data[0] = GdzCmd::POS_CTRL;
  union { float f; uint8_t b[4]; } pos_u;
  pos_u.f = pos;
  tx_frame.data[1] = pos_u.b[0];
  tx_frame.data[2] = pos_u.b[1];
  tx_frame.data[3] = pos_u.b[2];
  tx_frame.data[4] = pos_u.b[3];
  union { float f; uint8_t b[4]; } spd_u;
  spd_u.f = spd;
  tx_frame.data[5] = spd_u.b[0];
  tx_frame.data[6] = spd_u.b[1];
  tx_frame.data[7] = spd_u.b[2];

  can_->transmit(tx_frame);
  response_count_++;
}

// ============================================================
// 速度控制 (0xC1)
// ============================================================
void StwMotorDriver::motor_spd_cmd(float spd) {
  spd = std::clamp(spd, -limit_param_.SpdMax, limit_param_.SpdMax);

  can_frame tx_frame{};
  tx_frame.can_id  = motor_id_;
  tx_frame.can_dlc = 0x05;
  tx_frame.data[0] = GdzCmd::SPD_CTRL;
  int32_t spd_rpm = static_cast<int32_t>(spd * 60.0F / (2.0F * 3.14159265F) * 100.0F);
  tx_frame.data[1] = spd_rpm & 0xFF;
  tx_frame.data[2] = (spd_rpm >> 8) & 0xFF;
  tx_frame.data[3] = (spd_rpm >> 16) & 0xFF;
  tx_frame.data[4] = (spd_rpm >> 24) & 0xFF;

  can_->transmit(tx_frame);
  response_count_++;
}

// ============================================================
// 反馈解析
// ============================================================
void StwMotorDriver::can_rx_cbk(const can_frame& rx_frame) {
  response_count_ = 0;
  uint8_t cmd = rx_frame.data[0];
  switch (cmd) {
    case GdzCmd::READ_MT:       parse_0xf1_response(rx_frame); break;
    case GdzCmd::READ_STATUS:
      error_id_ = rx_frame.data[6];
      if (error_id_ != 0)
        logger_->error("STW motor {} fault 0x{:02X}", motor_id_, static_cast<unsigned>(error_id_));
      break;
    default: break;
  }
}

// 0xF1 运控实时数据 — 位置+速度 (按官方Demo: data[1:2]=位置uint16)
void StwMotorDriver::parse_0xf1_response(const can_frame& rx_frame) {
  uint16_t pos_raw = (static_cast<uint16_t>(rx_frame.data[1]) << 8) | rx_frame.data[2];
  motor_pos_ = -((static_cast<float>(pos_raw) / 65535.0F) * (2.0F * limit_param_.PosMax)
               - limit_param_.PosMax + static_cast<float>(motor_zero_offset_));

  uint16_t spd_raw = (static_cast<uint16_t>(rx_frame.data[3]) << 4)
                   | ((rx_frame.data[4] & 0xF0) >> 4);
  motor_spd_ = -((static_cast<float>(spd_raw) / 4095.0F) * (2.0F * limit_param_.SpdMax)
               - limit_param_.SpdMax);
}

// 状态刷新 — 0xF1由MIT帧自动触发, 无需主动查询
void StwMotorDriver::refresh_motor_status() {}

// 其他接口
void StwMotorDriver::set_motor_control_mode(uint8_t /*mode*/) {}

void StwMotorDriver::clear_motor_error() {
  can_frame tx_frame{};
  tx_frame.can_id  = motor_id_;
  tx_frame.can_dlc = 0x01;
  tx_frame.data[0] = GdzCmd::CLEAR_FAULT;
  can_->transmit(tx_frame);
  response_count_++;
}

bool StwMotorDriver::set_motor_zero() {
  can_frame tx_frame{};
  tx_frame.can_id  = motor_id_;
  tx_frame.can_dlc = 0x01;
  tx_frame.data[0] = GdzCmd::SET_ORIGIN;  // 0xB1
  can_->transmit(tx_frame);
  Timer::sleep_for(setup_sleep_time);
  return true;
}

bool StwMotorDriver::write_motor_flash() { return true; }

void StwMotorDriver::get_motor_param(uint8_t param_cmd) {
  can_frame tx_frame{};
  tx_frame.can_id  = motor_id_;
  tx_frame.can_dlc = 0x01;
  tx_frame.data[0] = GdzCmd::READ_VER;
  can_->transmit(tx_frame);
  response_count_++;
  (void)param_cmd;
}
