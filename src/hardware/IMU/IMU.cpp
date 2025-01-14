#include "hardware/IMU/IMU.hpp"

namespace lemlib {
int IMU::setGyroScalar(Number scalar) {
    m_gyroScalar = scalar;
    // Always returns 0 because the scalar setter is not dependent on hardware
    return 0;
}

Number IMU::getGyroScalar() {
    // Never returns an error because the scalar getter is not dependent on hardware
    return m_gyroScalar;
}
} // namespace lemlib