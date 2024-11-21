#include "hardware/Encoder/V5RotationSensor.hpp"
#include "hardware/Port.hpp"
#include "pros/rotation.h"
#include <limits.h>

namespace lemlib {
V5RotationSensor::V5RotationSensor(pros::Rotation encoder)
    : m_port(encoder.get_port()),
      m_reversed(encoder.get_port()) {
    pros::c::rotation_set_reversed(m_port, m_reversed);
}

V5RotationSensor::V5RotationSensor(ReversibleSmartPort port)
    : m_port(abs(port)),
      m_reversed(port < 0) {
    pros::c::rotation_set_reversed(m_port, m_reversed);
}

V5RotationSensor::V5RotationSensor(SmartPort port, bool reversed)
    : m_port(port),
      m_reversed(reversed) {
    pros::c::rotation_set_reversed(m_port, m_reversed);
}

int V5RotationSensor::isConnected() {
    if (pros::c::rotation_set_reversed(m_port, m_reversed) == INT_MAX) return 0;
    if (pros::c::rotation_get_angle(m_port) == INT32_MAX) return 0;
    else return 1;
}

Angle V5RotationSensor::getAngle() {
    if (pros::c::rotation_set_reversed(m_port, m_reversed) == INT_MAX) return from_stRot(INFINITY);
    const int32_t raw = pros::c::rotation_get_position(m_port);
    if (raw == INT_MAX) return from_stRot(INFINITY);
    const Angle angle = from_stDeg(double(raw) / 100);
    return angle + m_offset;
}

int V5RotationSensor::setAngle(Angle angle) {
    if (pros::c::rotation_set_reversed(m_port, m_reversed) == INT_MAX) return INT_MAX;
    // requestedAngle = pos + offset
    // offset = requestedAngle - raw
    const int32_t raw = pros::c::rotation_get_position(m_port);
    if (raw == INT_MAX) return INT_MAX;
    const Angle position = from_stDeg(double(raw) / 100);
    m_offset = angle - position;
    return 0;
}

int V5RotationSensor::isReversed() const { return m_reversed; }

int V5RotationSensor::setReversed(bool reversed) {
    m_reversed = reversed;
    return pros::c::rotation_set_reversed(m_port, m_reversed);
}
} // namespace lemlib