#include "encos_motor_driver.hpp"

#include <cstdint>

// 电机 id 和 零点都先设置好

ENCOS_Limit_Param encos_limit_param[ENCOS_Num_Of_Model] = {
    {12.5, 18.0, 30.0, -30.0, 30.0, 500.0, 5.0},
};

namespace {

constexpr uint8_t ENCOS_MODE_MIT = 0x00;
constexpr uint8_t ENCOS_MODE_BRAKE = 0x04;

float decode_temp(uint8_t raw_temp) {
    return (static_cast<float>(raw_temp) - 50.0f) / 2.0f;
}

}  // namespace

EncosMotorDriver::EncosMotorDriver(uint16_t motor_id, const std::string& interface_type, const std::string& can_interface,
                                   ENCOS_Motor_Model motor_model, double motor_zero_offset)
    : MotorDriver(), motor_model_(motor_model), can_(MotorsSocketCAN::get(can_interface)) {
    if (interface_type != "can") {
        throw std::runtime_error("ENCOS driver only support CAN interface");
    }
    motor_id_ = motor_id;
    limit_param_ = encos_limit_param[motor_model_];
    can_interface_ = can_interface;
    motor_zero_offset_ = motor_zero_offset;
    CanCbkFunc can_callback = std::bind(&EncosMotorDriver::can_rx_cbk, this, std::placeholders::_1);
    can_->add_can_callback(can_callback, motor_id_);
}

EncosMotorDriver::~EncosMotorDriver() { can_->remove_can_callback(motor_id_); }

void EncosMotorDriver::lock_motor() {
    can_frame tx_frame;
    tx_frame.can_id = motor_id_;
    tx_frame.can_dlc = 0x02;

    tx_frame.data[0] = static_cast<uint8_t>((ENCOS_MODE_BRAKE << 5) | 0x00);
    tx_frame.data[1] = 0x00;

    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void EncosMotorDriver::unlock_motor() {
    can_frame tx_frame;
    tx_frame.can_id = motor_id_;
    tx_frame.can_dlc = 0x02;

    tx_frame.data[0] = static_cast<uint8_t>((ENCOS_MODE_BRAKE << 5) | 0x00);
    tx_frame.data[1] = 0x01;

    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

uint8_t EncosMotorDriver::init_motor() {
    // set_motor_response_mode();
    Timer::sleep_for(setup_sleep_time);
    // EncosMotorDriver::unlock_motor();
    set_motor_control_mode(MIT);
    Timer::sleep_for(setup_sleep_time);
      
    return error_id_;
}

void EncosMotorDriver::deinit_motor() {
    // EncosMotorDriver::lock_motor();
    Timer::sleep_for(normal_sleep_time);
}

bool EncosMotorDriver::write_motor_flash() {
    logger_->warn("ENCOS write_motor_flash is not implemented yet");
    return false;
}

bool EncosMotorDriver::set_motor_zero() {
    logger_->warn("ENCOS set_motor_zero is intentionally left empty for now");
    return false;
}

void EncosMotorDriver::can_rx_cbk(const can_frame& rx_frame) {
    {
        response_count_ = 0;
    }
    const uint8_t frame_type = (rx_frame.data[0] >> 5) & 0x07;
    error_id_ = rx_frame.data[0] & 0x1F;

    if (error_id_ > 0 && logger_) {
        logger_->error(
            "can_interface: {0}\tmotor_id: {1}\terror_id: 0x{2:x}",
            can_interface_,
            motor_id_,
            static_cast<uint32_t>(error_id_));
    }

    switch (frame_type) {
        case 0x01: {
            const uint16_t pos_int = (static_cast<uint16_t>(rx_frame.data[1]) << 8) | rx_frame.data[2];
            const uint16_t spd_int = (static_cast<uint16_t>(rx_frame.data[3]) << 4) | ((rx_frame.data[4] & 0xF0) >> 4);
            const uint16_t cur_int = (static_cast<uint16_t>(rx_frame.data[4] & 0x0F) << 8) | rx_frame.data[5];

            motor_pos_ =
                range_map(pos_int, uint16_t(0), bitmax<uint16_t>(16), -limit_param_.PosMax, limit_param_.PosMax) +
                motor_zero_offset_;
            motor_spd_ =
                range_map(spd_int, uint16_t(0), bitmax<uint16_t>(12), -limit_param_.SpdMax, limit_param_.SpdMax);
            motor_current_ =
                range_map(cur_int, uint16_t(0), bitmax<uint16_t>(12), limit_param_.CurMin, limit_param_.CurMax);
            motor_temperature_ = decode_temp(rx_frame.data[6]);
            mos_temperature_ = rx_frame.data[7];
            break;
        }
        default:
            break;
    }
}

void EncosMotorDriver::get_motor_param(uint8_t param_cmd) {
    (void)param_cmd;
    logger_->warn("ENCOS MIT-only driver does not support parameter query");
}

void EncosMotorDriver::set_motor_response_mode() {
    can_frame tx_frame;
    tx_frame.can_id = 0x7FF;
    tx_frame.can_dlc = 0x04;

    tx_frame.data[0] = static_cast<uint8_t>(motor_id_ >> 8);
    tx_frame.data[1] = static_cast<uint8_t>(motor_id_ & 0xFF);
    tx_frame.data[2] = 0x00;
    tx_frame.data[3] = 0x02;

    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void EncosMotorDriver::motor_pos_cmd(float pos, float spd, bool ignore_limit) {
    (void)pos;
    (void)spd;
    (void)ignore_limit;
    logger_->warn("ENCOS MIT-only driver does not support position mode");
}

void EncosMotorDriver::motor_spd_cmd(float spd) {
    (void)spd;
    logger_->warn("ENCOS MIT-only driver does not support speed mode");
}

void EncosMotorDriver::motor_mit_cmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) {
    if (motor_control_mode_ != MIT) {
        motor_control_mode_ = MIT;
    }

    uint16_t p, v, kp, kd, t;
    can_frame tx_frame;

    f_p -= motor_zero_offset_;
    f_p = limit(f_p, -limit_param_.PosMax, limit_param_.PosMax);
    f_v = limit(f_v, -limit_param_.SpdMax, limit_param_.SpdMax);
    f_kp = limit(f_kp, 0.0f, limit_param_.OKpMax);
    f_kd = limit(f_kd, 0.0f, limit_param_.OKdMax);
    f_t = limit(f_t, -limit_param_.TauMax, limit_param_.TauMax);

    p = range_map(f_p, -limit_param_.PosMax, limit_param_.PosMax, uint16_t(0), bitmax<uint16_t>(16));
    v = range_map(f_v, -limit_param_.SpdMax, limit_param_.SpdMax, uint16_t(0), bitmax<uint16_t>(12));
    kp = range_map(f_kp, 0.0f, limit_param_.OKpMax, uint16_t(0), bitmax<uint16_t>(12));
    kd = range_map(f_kd, 0.0f, limit_param_.OKdMax, uint16_t(0), bitmax<uint16_t>(12));
    t = range_map(f_t, -limit_param_.TauMax, limit_param_.TauMax, uint16_t(0), bitmax<uint16_t>(12));

    tx_frame.can_id = motor_id_;
    tx_frame.can_dlc = 0x08;

    const uint64_t payload =
        (static_cast<uint64_t>(ENCOS_MODE_MIT) << 61) |
        (static_cast<uint64_t>(kp & 0x0FFF) << 49) |
        (static_cast<uint64_t>(kd & 0x01FF) << 40) |
        (static_cast<uint64_t>(p) << 24) |
        (static_cast<uint64_t>(v & 0x0FFF) << 12) |
        static_cast<uint64_t>(t & 0x0FFF);

    for (int index = 0; index < 8; ++index) {
        tx_frame.data[index] = static_cast<uint8_t>((payload >> ((7 - index) * 8)) & 0xFF);
    }

    can_->transmit(tx_frame);
    {
        response_count_++;
    }
}

void EncosMotorDriver::estop(float kd) {
    motor_mit_cmd(0.0f, 0.0f, 0.0f, kd, 0.0f);
}

void EncosMotorDriver::set_motor_control_mode(uint8_t motor_control_mode) {
    if (motor_control_mode != MIT) {
        logger_->warn("ENCOS driver only supports MIT mode, forcing MIT");
    }
    motor_control_mode_ = MIT;
}

void EncosMotorDriver::set_motor_zero_encos() {
    logger_->warn("ENCOS zero-point command is intentionally not implemented yet");
}

void EncosMotorDriver::clear_motor_error_encos() {
    logger_->warn("ENCOS clear_motor_error is not defined in the current driver");
}

void EncosMotorDriver::write_register_encos(uint8_t index, int32_t value) {
    (void)index;
    (void)value;
    logger_->warn("ENCOS write_register is not implemented yet");
}

void EncosMotorDriver::save_register_encos() {
    logger_->warn("ENCOS save_register is not implemented yet");
}




void EncosMotorDriver::refresh_motor_status() {
    // MIT mode returns status in the response frame, so there is nothing extra to poll here.
}

void EncosMotorDriver::clear_motor_error() {
    clear_motor_error_encos();
}
