#include "imu_driver.hpp"

#include <stdexcept>

#include "driver/yis320_imu_driver.hpp"

std::shared_ptr<IMUDriver> IMUDriver::create_imu(const std::string& interface_type, const std::string& interface,
                                                const std::string& imu_type, const int baudrate) {
    if (imu_type == "YIS320") {
        return std::make_shared<Yis320IMUDriver>(interface_type, interface, baudrate);
    }
    else {
        throw std::runtime_error("IMU type not supported: " + imu_type);
    }
}
