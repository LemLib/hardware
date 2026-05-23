#include "hardware/Distance/V5DistanceSensor.hpp"
#include "hardware/util.hpp"
#include <limits.h>
#include <mutex>
#include <cmath>

namespace lemlib {
V5DistanceSensor::V5DistanceSensor(SmartPort port)
    : m_port(port) {}

V5DistanceSensor::V5DistanceSensor(const V5DistanceSensor& other)
    : m_port(other.m_port),
      m_offset(other.m_offset) {}

V5DistanceSensor V5DistanceSensor::from_pros_dist(pros::Distance distanceSensor) {
    return V5DistanceSensor {{distanceSensor.get_port(), runtime_check_port}};
}

int32_t V5DistanceSensor::isConnected() const {
    if (pros::c::distance_get(m_port) == INT_MAX) return 0;
    else return 1;
}

Length V5DistanceSensor::getDistance() const {
    std::lock_guard lock(m_mutex);
    const int32_t raw = pros::c::distance_get(m_port);
    if (raw == INT_MAX) return from_in(INFINITY);
    // the distance sensor returns mm
    const Length distance = from_mm(raw);
    return distance + m_offset;
}

int32_t V5DistanceSensor::setOffset(Length offset) {
    const int32_t raw = pros::c::distance_get(m_port);
    if (raw == INT_MAX) return INT_MAX;
    m_offset = offset;
    return 0;
}
} // namespace lemlib