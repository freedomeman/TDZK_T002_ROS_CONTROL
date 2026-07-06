#pragma once

extern "C" {
#include "yis320_dec.h"
}

#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <vector>

#include "imu_driver.hpp"
#include "protocol/serial_port.hpp"

#define YIS320_DEG_TO_RAD  (0.01745329f)

class Yis320IMUDriver : public IMUDriver {
   public:
    Yis320IMUDriver(const std::string& interface_type, const std::string& interface, const int baudrate = 0);
    ~Yis320IMUDriver();

    void serial_rx_cbk(const uint8_t* data, size_t length);
    std::vector<float> get_ang_vel() override;
    std::vector<float> get_quat() override;
    std::vector<float> get_lin_acc() override;
    float get_temperature() override;

   private:
    int baudrate_;
    std::string interface_type_;
    std::string interface_;
    mutable std::shared_mutex imu_mutex_;
    std::shared_ptr<IMUSerialPort> serial_;
    yis320_raw_t raw_;
};

using YIS320IMUDriver = Yis320IMUDriver;
