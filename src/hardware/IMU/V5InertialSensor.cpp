#include "hardware/IMU/V5InertialSensor.hpp"
#include "hardware/Port.hpp"
#include "pros/imu.hpp"
#include <cstdint>
#include <mutex>

namespace lemlib {
V5InertialSensor::V5InertialSensor(SmartPort port, Number scalar)
    : IMU(scalar),
      m_imu(port) {}

V5InertialSensor::V5InertialSensor(const V5InertialSensor& other)
    : IMU(other.getGyroScalar()),
      m_imu(other.m_imu),
      m_offset(other.m_offset) {}

V5InertialSensor V5InertialSensor::from_pros_imu(pros::IMU imu, Number scalar) {
    return V5InertialSensor({imu.get_port(), runtime_check_port}, scalar);
}

SmartPort V5InertialSensor::getPort() const {
    std::lock_guard lock(m_mutex);
    SmartPort port(m_imu.get_port(), DynamicPort {});
    return port;
}

int32_t V5InertialSensor::calibrate() {
    std::lock_guard lock(m_mutex);
    m_offset = 0_stRot;
    return m_imu.reset();
}

int32_t V5InertialSensor::isCalibrated() const { return !m_imu.is_calibrating(); }

int32_t V5InertialSensor::isCalibrating() const { return m_imu.is_calibrating(); }

int32_t V5InertialSensor::isConnected() const { return m_imu.is_installed(); }

Angle V5InertialSensor::getRotation() const {
    std::lock_guard lock(m_mutex);
    const double result = m_imu.get_rotation();
    // check for errors
    if (result == INFINITY) return from_stDeg(INFINITY);
    return from_cDeg(result * m_gyroScalar) + m_offset;
}

int32_t V5InertialSensor::setRotation(Angle rotation) {
    std::lock_guard lock(m_mutex);
    Angle raw = this->getRotation();
    if (to_stRot(raw) == INFINITY) return INT32_MAX;
    else {
        m_offset = rotation - raw;
        return 0;
    }
}
} // namespace lemlib