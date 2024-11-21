#include "hardware/Imu/V5InertialSensor.hpp"
#include "hardware/Port.hpp"

namespace lemlib {
V5InertialSensor::V5InertialSensor(pros::Imu imu)
    : m_imu(imu) {}

V5InertialSensor::V5InertialSensor(SmartPort port)
    : m_imu(port) {}

int V5InertialSensor::calibrate() { return m_imu.reset(); }

int V5InertialSensor::isCalibrated() { return m_imu.is_calibrating(); }

int V5InertialSensor::isCalibrating() { return m_imu.is_calibrating(); }

int V5InertialSensor::isConnected() { return m_imu.is_installed(); }

Angle V5InertialSensor::getRotation() {
    const double result = m_imu.get_rotation();
    // check for errors
    if (result == INFINITY) return from_stDeg(INFINITY);
    return from_cDeg(result);
}

int V5InertialSensor::setRotation(Angle rotation) { return m_imu.set_rotation(to_cDeg(rotation)); }
} // namespace lemlib