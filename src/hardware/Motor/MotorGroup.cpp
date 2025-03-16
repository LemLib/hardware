#include "hardware/Motor/MotorGroup.hpp"
#include "hardware/Port.hpp"
#include "hardware/Motor/Motor.hpp"
#include "units/Angle.hpp"
#include "units/Temperature.hpp"
#include <climits>
#include <cmath>
#include <cstdint>
#include <errno.h>
#include <mutex>

namespace lemlib {
MotorGroup::MotorGroup(const std::initializer_list<ReversibleSmartPort>& ports, AngularVelocity outputVelocity)
    : m_outputVelocity(outputVelocity) {
    for (const auto port : ports) { m_motors.push_back({.port = port, .connectedLastCycle = true, .offset = 0_stDeg}); }
}

MotorGroup::MotorGroup(const MotorGroup& other)
    : m_brakeMode(other.getBrakeMode()),
      m_outputVelocity(other.getOutputVelocity()),
      m_motors(other.getMotorInfo()) {}

MotorGroup MotorGroup::from_pros_group(pros::MotorGroup group, AngularVelocity outputVelocity) {
    MotorGroup motor_group {{}, outputVelocity};
    const std::vector<std::int8_t> ports = group.get_port_all();
    for (const int port : ports) {
        motor_group.m_motors.push_back(
            {.port = ReversibleSmartPort {port, runtime_check_port}, .connectedLastCycle = true, .offset = 0_stDeg});
    }
    return motor_group;
}

int32_t MotorGroup::move(Number percent) {
    std::lock_guard lock(m_mutex);
    const std::vector<Motor> motors = getMotors();
    bool success = false;
    for (Motor motor : motors) {
        const int result = motor.move(percent);
        if (result == 0) success = true;
    }
    // as long as one motor moves successfully, return 0 (success)
    return success ? 0 : INT_MAX;
}

int32_t MotorGroup::moveVelocity(AngularVelocity velocity) {
    std::lock_guard lock(m_mutex);
    const std::vector<Motor> motors = getMotors();
    bool success = false;
    for (Motor motor : motors) {
        const int result = motor.moveVelocity(velocity);
        if (result == 0) success = true;
    }
    // as long as one motor moves successfully, return 0 (success)
    return success ? 0 : INT_MAX;
}

int32_t MotorGroup::brake() {
    std::lock_guard lock(m_mutex);
    const std::vector<Motor> motors = getMotors();
    bool success = false;
    for (Motor motor : motors) {
        const int result = motor.brake();
        if (result == 0) success = true;
    }
    // as long as one motor brakes successfully, return 0 (success)
    return success ? 0 : INT_MAX;
}

int32_t MotorGroup::setBrakeMode(BrakeMode mode) {
    std::lock_guard lock(m_mutex);
    m_brakeMode = mode;
    getMotors(); // even though we don't use this, we call it anyway for brake mode setting and disconnect handling
    return 0;
}

BrakeMode MotorGroup::getBrakeMode() const {
    std::lock_guard lock(m_mutex);
    getMotors(); // even though we don't use this, we call it anyway for brake mode setting and disconnect handling
    return m_brakeMode;
}

int32_t MotorGroup::isConnected() const {
    std::lock_guard lock(m_mutex);
    const std::vector<Motor> motors = getMotors();
    for (Motor motor : motors) {
        const int result = motor.isConnected();
        if (result == 1) return true;
    }
    // if no motors are connected, return 0
    return 0;
}

Angle MotorGroup::getAngle() const {
    std::lock_guard lock(m_mutex);
    const std::vector<Motor> motors = getMotors();
    // get the average angle of all motors in the group
    Angle angle = 0_stDeg;
    int errors = 0;
    for (Motor motor : motors) {
        // get angle
        const Angle result = motor.getAngle();
        if (result == from_stDeg(INFINITY)) {
            errors++;
            continue;
        };
        // add to sum
        angle += result;
    }
    // if no motors are connected, return INFINITY
    if (errors == motors.size()) return from_stDeg(INFINITY);
    // otherwise, return the average angle
    return angle / (motors.size() - errors);
}

int32_t MotorGroup::setAngle(Angle angle) {
    std::lock_guard lock(m_mutex);
    const std::vector<Motor> motors = getMotors();
    bool success = false;
    for (Motor motor : motors) {
        const int result = motor.setAngle(angle);
        if (result == 0) success = true;
        const Angle offset = motor.getOffset();
        // set the offset of that motor
        for (MotorInfo& info : m_motors) {
            if (info.port == motor.getPort()) info.offset = offset;
        }
    }
    // as long as one motor sets the angle successfully, return 0 (success)
    return success ? 0 : INT_MAX;
}

Current MotorGroup::getCurrentLimit() const {
    std::lock_guard lock(m_mutex);
    const std::vector<Motor> motors = getMotors();
    Current total = 0_amp;
    int errors = 0;
    // find the total current limit
    for (Motor motor : motors) {
        Current result = motor.getCurrentLimit();
        if (result.internal() == INFINITY) { // error checking
            errors += 1;
            continue;
        }
        total += result;
    }
    if (errors == motors.size()) return from_amp(INFINITY); // error checking
    return total;
}

int32_t MotorGroup::setCurrentLimit(Current limit) {
    std::lock_guard lock(m_mutex);
    const std::vector<Motor> motors = getMotors();
    if (motors.size() == 0) return INT_MAX; // error handling
    // set current limits
    for (Motor motor : motors) {
        int result = motor.setCurrentLimit(limit / motors.size());
        if (result == INT_MAX) return setCurrentLimit(limit); // redo if there was a failure
    }
    return 0;
}

std::vector<Temperature> MotorGroup::getTemperatures() const {
    std::lock_guard lock(m_mutex);
    std::vector<Motor> motors = getMotors();
    std::vector<Temperature> temperatures;
    for (const Motor motor : motors) { temperatures.push_back(motor.getTemperature()); }
    return temperatures;
}

std::vector<Current> MotorGroup::getCurrents() const {
    std::lock_guard lock(m_mutex);
    std::vector<Motor> motors = getMotors();
    std::vector<Current> currents;
    for (const Motor motor : motors) { currents.push_back(motor.getCurrent()); }
    return currents;
}

std::vector<AngularVelocity> MotorGroup::getMotorCartridges() const {
    std::lock_guard lock(m_mutex);
    std::vector<Motor> motors = getMotors();
    std::vector<AngularVelocity> cartridges;
    for (const Motor motor : motors) { cartridges.push_back(motor.getMotorCartridge()); }
    return cartridges;
}

AngularVelocity MotorGroup::getActualVelocity() const {
    std::lock_guard lock(m_mutex);
    const std::vector<Motor> motors = getMotors();
    // get the average angle of all motors in the group
    AngularVelocity av = 0_rpm;
    int errors = 0;
    for (Motor motor : motors) {
        // get angle
        const AngularVelocity result = motor.getActualVelocity();
        if (result == from_rpm(INFINITY)) {
            errors++;
            continue;
        };
        // add to sum
        av += result;
    }
    // if no motors are connected, return INFINITY
    if (errors == motors.size()) return from_rpm(INFINITY);
    // otherwise, return the average angle
    return av / (motors.size() - errors);
}

// Always returns 0 because the velocity setter is not dependent on hardware and should never fail
int32_t MotorGroup::setOutputVelocity(AngularVelocity outputVelocity) {
    std::lock_guard lock(m_mutex);
    Angle angle = getAngle();
    m_outputVelocity = outputVelocity;
    setAngle(angle);
    return 0;
}

AngularVelocity MotorGroup::getOutputVelocity() const {
    std::lock_guard lock(m_mutex);
    return m_outputVelocity;
}

int32_t MotorGroup::getSize() const {
    std::lock_guard lock(m_mutex);
    const std::vector<Motor> motors = getMotors();
    int size = 0;
    for (Motor motor : motors)
        if (motor.isConnected()) size++;
    return size;
}

int32_t MotorGroup::addMotor(ReversibleSmartPort port) {
    std::lock_guard lock(m_mutex);
    // check that the motor isn't already part of the group
    for (const MotorInfo& info : m_motors) {
        // return an error if the motor is already added to the group
        if (std::abs(info.port) == std::abs(port)) {
            errno = EEXIST;
            return INT_MAX;
        }
    }
    // configure the motor
    const Angle offset = configureMotor(port);
    // add the motor to the group
    MotorInfo info {.port = port, .connectedLastCycle = offset == from_stRot(INFINITY), .offset = offset};
    m_motors.push_back(info);
    if (offset == from_stRot(INFINITY)) return INT_MAX;
    return 0;
}

int32_t MotorGroup::addMotor(Motor motor) {
    std::lock_guard lock(m_mutex);
    return addMotor(motor.getPort());
}

int32_t MotorGroup::addMotor(Motor motor, bool reversed) {
    std::lock_guard lock(m_mutex);
    // set the motor reversal
    motor.setReversed(reversed);
    return addMotor(motor);
}

void MotorGroup::removeMotor(ReversibleSmartPort port) {
    std::lock_guard lock(m_mutex);
    // remove the motor with the specified port
    const auto iterator = std::remove_if(m_motors.begin(), m_motors.end(), [&](MotorInfo m) { return m.port == port; });
    m_motors.erase(iterator);
}

void MotorGroup::removeMotor(Motor motor) { removeMotor(motor.getPort()); }

const std::vector<Motor> MotorGroup::getMotors() const {
    std::vector<Motor> motors;
    for (int i = 0; i < m_motors.size(); i++) {
        // create a temporary motor
        Motor motor(m_motors.at(i).port, m_outputVelocity);
        // set motor offset
        motor.setOffset(m_motors.at(i).offset);
        // check if the motor is connected
        const bool connected = motor.isConnected();
        // don't add the motor if it is not connected
        if (!connected) {
            m_motors.at(i).connectedLastCycle = false;
            continue;
        }
        // if the motor is connected, but wasn't the last time we checked, then configure it to prevent side
        // effects of reconnecting
        // don't add the motor if configuration fails
        if (!m_motors.at(i).connectedLastCycle) {
            if (std::isinf(to_stRot(configureMotor(m_motors.at(i).port)))) continue;
        }
        // check that the brake mode of the motor is correct
        BrakeMode mode = motor.getBrakeMode();
        if (mode != m_brakeMode) {
            if (motor.setBrakeMode(m_brakeMode) != 0) continue;
        } else if (mode == BrakeMode::INVALID) continue;
        // add the motor and set save it as connected
        m_motors.at(i).connectedLastCycle = true;
        motors.push_back(motor);
    }
    return motors;
}

const std::vector<MotorGroup::MotorInfo> MotorGroup::getMotorInfo() const {
    std::lock_guard lock(m_mutex);
    return m_motors;
}

Angle MotorGroup::configureMotor(ReversibleSmartPort port) const {
    // since this function is called in other MotorGroup member functions, this function can't call any other member
    // function, otherwise it would cause a recursion loop. This means that this function is ugly and complex, but at
    // least it means that the other functions can stay simple

    // this function does not return immediately when something goes wrong. Instead it will continue with the process to
    // add the motor, and the motor will automatically be reconfigured when it is working properly again
    // whether there was a failure or not is kept track of with this boolean
    bool success = true;
    Motor motor(port, m_outputVelocity);
    // set the motor's brake mode to whatever the first working motor's brake mode is
    for (MotorInfo info : m_motors) {
        Motor m(info.port, m_outputVelocity);
        const BrakeMode mode = m.getBrakeMode();
        if (mode == BrakeMode::INVALID) continue;
        if (m.setBrakeMode(mode) == 0) break;
        else {
            success = false;
            break;
        }
    }

    Angle angle = 0_stDeg;
    {
        // find all the working motors
        std::vector<Motor> motors;
        for (int i = 0; i < m_motors.size(); i++) {
            // temporary motor object
            Motor m(m_motors.at(i).port, m_outputVelocity);
            // set the offset of the motor
            m.setOffset(m_motors.at(i).offset);
            // check if the motor is connected
            const bool connected = m.isConnected();
            // check that the motor is not the motor we are configuring
            if (std::abs(m.getPort()) == std::abs(port)) continue;
            // don't add the motor if it is not connected
            if (!connected) continue;
            else motors.push_back(m);
        }

        // get the average angle of all motors in the group
        Angle tempAngle = 0_stDeg;
        int errors = 0;
        for (Motor m : motors) {
            // get angle
            const Angle result = m.getAngle();
            if (result == from_stDeg(INFINITY)) { // check for errors
                errors++;
                continue;
            }
            tempAngle += result;
        }
        // prevent divide by zero if all motors failed
        if (motors.size() != errors) angle = tempAngle / (motors.size() - errors);
    }

    // set the angle of the new motor
    const int result = motor.setAngle(angle);
    if (result == INT_MAX) return from_stRot(INFINITY); // check for errors
    if (success == true) return motor.getOffset();
    return from_stRot(INFINITY);
}
}; // namespace lemlib