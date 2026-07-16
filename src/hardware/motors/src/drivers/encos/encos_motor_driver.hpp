#pragma once

#include <atomic>
#include <string>

#include "motor_driver.hpp"
#include "protocol/can/socket_can.hpp"

enum ENCOSError {
    ENCOS_NO_ERROR = 0x00,
    ENCOS_OVER_VOLTAGE = 0x01,
    ENCOS_UNDER_VOLTAGE = 0x02,
    ENCOS_OVER_CURRENT = 0x03,
    ENCOS_MOS_OVER_TEMP = 0x09,
    ENCOS_COIL_OVER_TEMP = 0x0A,
    ENCOS_ENCODER_ERROR = 0x0B,
    ENCOS_OVERLOAD = 0x0F,
    ENCOS_COMM_LOST = 0x10,
    ENCOS_UNKNOWN_ERROR = 0xFF
};

enum ENCOS_Motor_Model {
    EC_A4310 = 0,
    ENCOS_Num_Of_Model
};

typedef struct {
    float PosMax;
    float SpdMax;
    float TauMax;
    float CurMin;
    float CurMax;
    float OKpMax;
    float OKdMax;
} ENCOS_Limit_Param;

class EncosMotorDriver : public MotorDriver {
   public:
    EncosMotorDriver(uint16_t motor_id, const std::string& interface_type, const std::string& can_interface,
                     ENCOS_Motor_Model motor_model, double motor_zero_offset = 0.0);
    ~EncosMotorDriver();

    virtual void lock_motor() override;
    virtual void unlock_motor() override;
    virtual uint8_t init_motor() override;
    virtual void deinit_motor() override;
    virtual bool set_motor_zero() override;
    virtual bool write_motor_flash() override;

    virtual void get_motor_param(uint8_t param_cmd) override;
    virtual void motor_pos_cmd(float pos, float spd, bool ignore_limit) override;
    virtual void motor_spd_cmd(float spd) override;
    virtual void motor_mit_cmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) override;
    virtual void estop(float kd) override;
    virtual void reset_motor_id() override {};
    virtual void set_motor_control_mode(uint8_t motor_control_mode) override;
    virtual int get_response_count() const override {
        return response_count_;
    }
    virtual void refresh_motor_status() override;
    virtual void clear_motor_error() override;

   private:
    std::atomic<int> response_count_{0};
    ENCOS_Motor_Model motor_model_;
    ENCOS_Limit_Param limit_param_;
    std::atomic<uint8_t> mos_temperature_{0};
    void set_motor_response_mode();
    void set_motor_zero_encos();
    void clear_motor_error_encos();
    void write_register_encos(uint8_t index, int32_t value);
    void save_register_encos();
    virtual void can_rx_cbk(const can_frame& rx_frame);
    std::shared_ptr<MotorsSocketCAN> can_;
    std::string can_interface_;
};
