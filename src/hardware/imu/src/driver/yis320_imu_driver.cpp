#include "yis320_imu_driver.hpp"

#include <cstring>
#include <functional>
#include <stdexcept>

Yis320IMUDriver::Yis320IMUDriver(const std::string& interface_type, const std::string& interface, const int baudrate)
    : IMUDriver(), baudrate_(baudrate), interface_type_(interface_type), interface_(interface) {
    memset(&raw_, 0, sizeof(raw_));

    if (interface_type_ == "serial") {
        serial_ = IMUSerialPort::open(interface_, baudrate_);
        IMUSerialPort::SerialCbkFunc serial_callback =
            std::bind(&Yis320IMUDriver::serial_rx_cbk, this, std::placeholders::_1, std::placeholders::_2);
        serial_->set_serial_callback(serial_callback);
    } else {
        throw std::runtime_error("Yis320 driver only supports SERIAL interface");
    }
}

Yis320IMUDriver::~Yis320IMUDriver() {
    if (interface_type_ == "serial" && serial_) {
        serial_->close();
    }
}

void Yis320IMUDriver::serial_rx_cbk(const uint8_t* data, size_t length) {
    std::unique_lock<std::shared_mutex> lock(imu_mutex_);

    for (size_t i = 0; i < length; i++) {
        int ret = yis320_input(&raw_, data[i]);
        if (ret <= 0) {
            continue;
        }

        const yis320_packet_t& packet = raw_.packet;

        if (packet.data_bitmap & YIS320_BMAP_QUAT) {
            quat_[0] = packet.quat[0];
            quat_[1] = packet.quat[1];
            quat_[2] = packet.quat[2];
            quat_[3] = packet.quat[3];
        }

        if (packet.data_bitmap & YIS320_BMAP_GYR) {
            ang_vel_[0] = packet.gyr[0] * YIS320_DEG_TO_RAD;
            ang_vel_[1] = packet.gyr[1] * YIS320_DEG_TO_RAD;
            ang_vel_[2] = packet.gyr[2] * YIS320_DEG_TO_RAD;
        }

        if (packet.data_bitmap & YIS320_BMAP_ACC) {
            lin_acc_[0] = packet.acc[0];
            lin_acc_[1] = packet.acc[1];
            lin_acc_[2] = packet.acc[2];
        }

        if (packet.data_bitmap & YIS320_BMAP_TEMPERATURE) {
            temperature_ = packet.temperature;
        }
    }
}

std::vector<float> Yis320IMUDriver::get_ang_vel() {
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return ang_vel_;
}

std::vector<float> Yis320IMUDriver::get_quat() {
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return quat_;
}

std::vector<float> Yis320IMUDriver::get_lin_acc() {
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return lin_acc_;
}

float Yis320IMUDriver::get_temperature() {
    std::shared_lock<std::shared_mutex> lock(imu_mutex_);
    return temperature_;
}
