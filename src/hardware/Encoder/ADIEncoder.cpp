#include "hardware/Encoder/ADIEncoder.hpp"
#include "hardware/Port.hpp"
#include <cmath>
#include <limits.h>

namespace lemlib {
ADIEncoder::ADIEncoder(pros::adi::Encoder encoder)
    : m_encoder(encoder) {}

ADIEncoder::ADIEncoder(ADIPort topPort, ADIPort bottomPort, bool reversed)
    : m_encoder(pros::adi::Encoder(topPort, bottomPort, reversed)) {}

ADIEncoder::ADIEncoder(SmartPort expanderPort, ADIPort topPort, ADIPort bottomPort, bool reversed)
    : m_encoder({expanderPort, topPort, bottomPort}, reversed) {}

int ADIEncoder::isConnected() {
    // it's not possible to check if the ADIEncoder is connected, so we just return 1 to indicate that it is
    // we do run a simple test however to check if the ports are valid
    if (m_encoder.get_value() == INT_MAX) {
        errno = ENODEV;
        return INT_MAX;
    }
    // return 1 to indicate that the encoder is "connected"
    return 1;
}

Angle ADIEncoder::getAngle() {
    const int raw = m_encoder.get_value();
    // check for errors
    if (raw == INT_MAX) {
        errno = ENODEV;
        return Angle(INFINITY);
    }
    // return the angle
    return from_stDeg(raw) + m_offset;
}

int ADIEncoder::setAngle(Angle angle) {
    // the Vex SDK does not support setting the relative angle of an ADI encoder to a specific value
    // but we can overcome this limitation by resetting the relative angle to zero and saving an offset
    m_offset = angle;
    const int result = m_encoder.reset();
    // check for errors
    if (result == INT_MAX) {
        errno = ENODEV;
        return INT_MAX;
    }
    // return 0 on success
    return 0;
}
} // namespace lemlib