#include "hardware/Motor/Motor.hpp"
#include "hardware/Port.hpp"
#include "hardware/util.hpp"
#include "units/Angle.hpp"
#include "pros/device.h"
#include "pros/error.h"
#include "pros/motors.h"
#include "units/Temperature.hpp"
#include "units/units.hpp"
#include <cstdint>
#include <mutex>

namespace lemlib {
Motor::Motor(ReversibleSmartPort port, AngularVelocity outputVelocity)
    : m_port(port),
      m_outputVelocity(outputVelocity) {}

Motor::Motor(const Motor& other)
    : m_port(other.m_port),
      m_outputVelocity(other.m_outputVelocity),
      m_offset(other.m_offset) {}

Motor Motor::from_pros_motor(const pros::Motor motor, AngularVelocity outputVelocity) {
    return Motor {{motor.get_port(), runtime_check_port}, outputVelocity};
}

pros::motor_brake_mode_e_t brakeModeToMotorBrake(BrakeMode mode) {
    // MotorBrake is identical to lemlib::BrakeMode, except for its name and lemlib uses an enum class for type
    // safety
    switch (mode) {
        case BrakeMode::COAST: return pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_COAST;
        case BrakeMode::BRAKE: return pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE;
        case BrakeMode::HOLD: return pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD;
        default: return pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_INVALID;
    }
}

BrakeMode motorBrakeToBrakeMode(pros::motor_brake_mode_e_t mode) {
    // MotorBrake is identical to lemlib::BrakeMode, except for its name and lemlib uses an enum class for type
    // safety
    switch (mode) {
        case pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_COAST: return BrakeMode::COAST;
        case pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE: return BrakeMode::BRAKE;
        case pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD: return BrakeMode::HOLD;
        default: return BrakeMode::INVALID;
    }
}

int32_t Motor::move(Number percent) {
    // the V5 and EXP motors have different voltage caps, so we need to scale based on the motor type
    // V5 motors have their voltage capped at 12v, while EXP motors have their voltage capped at 7.2v
    // but they have the same max velocity, so we can scale the percent power based on the motor type
    switch (getType()) {
        case (MotorType::V5): return convertStatus(pros::c::motor_move_voltage(m_port, percent.internal() * 12000));
        case (MotorType::EXP): return convertStatus(pros::c::motor_move_voltage(m_port, percent.internal() * 7200));
        default: return INT_MAX;
    }
}

int32_t Motor::moveVelocity(AngularVelocity velocity) {
    std::lock_guard lock(m_mutex);
    // vexos will behave differently depending on the cartridge of the motor
    // pros uses an integer value to represent the rpm of the motor
    const pros::motor_gearset_e_t mode = pros::c::motor_get_gearing(m_port);
    AngularVelocity motorCartridge = 0_rpm;
    switch (mode) {
        case (pros::E_MOTOR_GEARSET_06): {
            motorCartridge = 600_rpm;
            break;
        }
        case (pros::E_MOTOR_GEARSET_18): {
            motorCartridge = 200_rpm;
            break;
        }
        case (pros::E_MOTOR_GEARSET_36): {
            motorCartridge = 100_rpm;
            break;
        }
        default: return INT_MAX;
    }
    const int out = to_rpm(units::round(velocity * (motorCartridge / m_outputVelocity), rpm));
    return convertStatus(pros::c::motor_move_velocity(m_port, out));
}

int32_t Motor::brake() {
    std::lock_guard lock(m_mutex);
    return convertStatus(pros::c::motor_brake(m_port));
}

int32_t Motor::setBrakeMode(BrakeMode mode) {
    std::lock_guard lock(m_mutex);
    if (mode == BrakeMode::INVALID) {
        errno = EINVAL;
        return INT_MAX;
    }
    return convertStatus(pros::c::motor_set_brake_mode(m_port, brakeModeToMotorBrake(mode)));
}

BrakeMode Motor::getBrakeMode() const {
    std::lock_guard lock(m_mutex);
    return motorBrakeToBrakeMode(pros::c::motor_get_brake_mode(m_port));
}

int32_t Motor::isConnected() const {
    return pros::c::get_plugged_type(abs(m_port)) == pros::c::v5_device_e_t::E_DEVICE_MOTOR;
}

Angle Motor::getAngle() const {
    std::lock_guard lock(m_mutex);
    // get the number of encoder ticks
    const int ticks = pros::c::motor_get_raw_position(m_port, NULL);
    if (ticks == INT_MAX) return from_stRot(INFINITY);
    // get the number of times the motor rotated
    const Angle raw = from_stRot(ticks / 50.0);
    // calculate position after using the gear ratio
    const Angle position = raw * (m_outputVelocity / 3600_rpm);
    // return position + offset
    return position + m_offset;
}

int32_t Motor::setAngle(Angle angle) {
    std::lock_guard lock(m_mutex);
    // get the raw position
    const int ticks = pros::c::motor_get_raw_position(m_port, NULL);
    if (ticks == INT_MAX) return INT_MAX;
    // get the number of times the internal motor rotated
    const Angle raw = from_stRot(ticks / 50.0);
    // calculate position after using the gear ratio
    const Angle position = raw * (m_outputVelocity / 3600_rpm);
    // calculate offset
    m_offset = angle - position;
    return 0;
}

Angle Motor::getOffset() const {
    std::lock_guard lock(m_mutex);
    return m_offset;
}

int32_t Motor::setOffset(Angle offset) {
    std::lock_guard lock(m_mutex);
    m_offset = offset;
    return 0;
}

MotorType Motor::getType() const {
    std::lock_guard lock(m_mutex);
    // there is no exposed api to get the motor type
    // while the memory address of the function has been found through reverse engineering,
    // it may break between VEXos updates. Instead, we see if we can change the cartridge to something other
    // than the green cartridge, which is only possible on the V5 motor
    const pros::motor_gearset_e_t oldCart = pros::c::motor_get_gearing(m_port);
    const int result = pros::c::motor_set_gearing(m_port, pros::motor_gearset_e_t::E_MOTOR_GEAR_RED);
    // check for errors
    if (oldCart == pros::motor_gearset_e_t::E_MOTOR_GEARSET_INVALID) return MotorType::INVALID;
    if (result == INT_MAX) return MotorType::INVALID;
    // check if the gearing changed or not
    const pros::motor_gearset_e_t newCart = pros::c::motor_get_gearing(m_port);
    if (newCart == pros::motor_gearset_e_t::E_MOTOR_GEARSET_INVALID) return MotorType::INVALID;
    if (newCart != pros::motor_gearset_e_t::E_MOTOR_GEAR_GREEN) {
        // set the cartridge back to its original value
        if (pros::c::motor_set_gearing(m_port, oldCart) == INT_MAX) return MotorType::INVALID;
        else return MotorType::V5;
    } else return MotorType::EXP;
}

int32_t Motor::isReversed() const {
    std::lock_guard lock(m_mutex);
    // technically this returns an int, but as long as you only pass 0 to the index its impossible for it to return an
    // error. This is because we keep track of whether the motor is reversed or not through the sign of its port
    return m_port < 0;
}

int32_t Motor::setReversed(bool reversed) {
    std::lock_guard lock(m_mutex);
    // technically this returns an int, but as long as you only pass 0 to the index its impossible for it to return an
    // error. This is because we keep track of whether the motor is reversed or not through the sign of its port
    m_port = m_port.set_reversed(reversed);
    return 0;
}

ReversibleSmartPort Motor::getPort() const {
    std::lock_guard lock(m_mutex);
    return m_port;
}

Current Motor::getCurrentLimit() const {
    std::lock_guard lock(m_mutex);
    const Current result = from_amp(pros::c::motor_get_current_limit(m_port));
    if (result.internal() == INT32_MAX) return from_amp(INFINITY); // error checking
    return result;
}

int32_t Motor::setCurrentLimit(Current limit) {
    std::lock_guard lock(m_mutex);
    return pros::c::motor_set_current_limit(m_port, to_amp(limit) * 1000);
}

Temperature Motor::getTemperature() const {
    const Temperature result = units::from_celsius(pros::c::motor_get_temperature(m_port));
    if (result.internal() == INFINITY) return result; // error checking
    return result;
}

Current Motor::getCurrent() const {
    const Current result = from_amp(pros::c::motor_get_current_draw(m_port) / 1000.0);
    if (result.internal() == INFINITY) return result; // error checking
    return result;
}

// Always returns 0 because the velocity setter is not dependent on hardware and should never fail
int32_t Motor::setOutputVelocity(AngularVelocity outputVelocity) {
    std::lock_guard lock(m_mutex);
    Angle angle = getAngle();
    m_outputVelocity = outputVelocity;
    setAngle(angle);
    return 0;
}

AngularVelocity Motor::getOutputVelocity() const {
    std::lock_guard lock(m_mutex);
    return m_outputVelocity;
}

AngularVelocity Motor::getMotorCartridge() const {
    std::lock_guard lock(m_mutex);
    const pros::motor_gearset_e_t mode = pros::c::motor_get_gearing(m_port);
    switch (mode) {
        case (pros::E_MOTOR_GEARSET_06): {
            return 600_rpm;
            break;
        }
        case (pros::E_MOTOR_GEARSET_18): {
            return 200_rpm;
            break;
        }
        case (pros::E_MOTOR_GEARSET_36): {
            return 100_rpm;
            break;
        }
        default: return from_rpm(INFINITY);
    }
}

AngularVelocity Motor::getActualVelocity() const {
    std::lock_guard lock(m_mutex);
    const double v = pros::c::motor_get_actual_velocity(m_port);
    if (v == PROS_ERR_F) return from_rpm(INFINITY);
    const pros::motor_gearset_e_t mode = pros::c::motor_get_gearing(m_port);
    AngularVelocity cartridgeSpeed = 0_rpm;
    switch (mode) {
        case (pros::E_MOTOR_GEARSET_06): {
            cartridgeSpeed = 600_rpm;
            break;
        }
        case (pros::E_MOTOR_GEARSET_18): {
            cartridgeSpeed = 200_rpm;
            break;
        }
        case (pros::E_MOTOR_GEARSET_36): {
            cartridgeSpeed = 100_rpm;
            break;
        }
        default: return from_rpm(INFINITY);
    }

    return from_rpm(v) * (m_outputVelocity / cartridgeSpeed);
}

} // namespace lemlib