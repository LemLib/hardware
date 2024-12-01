#include "hardware/Motor/Motor.hpp"
#include "hardware/util.hpp"
#include "units/Angle.hpp"
#include "pros/device.h"
#include "pros/motors.h"
#include "units/Temperature.hpp"
#include "units/units.hpp"
#include <cmath>
#include <cstdint>

namespace lemlib {
Motor::Motor(int port, AngularVelocity outputVelocity)
    : m_port(port),
      m_outputVelocity(outputVelocity) {}

Motor::Motor(uint8_t port, bool reversed, AngularVelocity outputVelocity)
    : m_port(reversed ? -port : port),
      m_outputVelocity(outputVelocity) {}

Motor::Motor(const pros::Motor motor, AngularVelocity outputVelocity)
    : m_port(motor.get_port()),
      m_outputVelocity(outputVelocity) {}

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

int Motor::move(double percent) {
    // the V5 and EXP motors have different voltage caps, so we need to scale based on the motor type
    // V5 motors have their voltage capped at 12v, while EXP motors have their voltage capped at 7.2v
    // but they have the same max velocity, so we can scale the percent power based on the motor type
    switch (getType()) {
        case (MotorType::V5): return convertStatus(pros::c::motor_move_voltage(m_port, percent * 12000));
        case (MotorType::EXP): return convertStatus(pros::c::motor_move_voltage(m_port, percent * 7200));
        default: return INT_MAX;
    }
}

int Motor::moveVelocity(AngularVelocity velocity) {
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

int Motor::brake() { return convertStatus(pros::c::motor_brake(m_port)); }

int Motor::setBrakeMode(BrakeMode mode) {
    if (mode == BrakeMode::INVALID) {
        errno = EINVAL;
        return INT_MAX;
    }
    return convertStatus(pros::c::motor_set_brake_mode(m_port, brakeModeToMotorBrake(mode)));
}

BrakeMode Motor::getBrakeMode() const { return motorBrakeToBrakeMode(pros::c::motor_get_brake_mode(m_port)); }

int Motor::isConnected() { return pros::c::get_plugged_type(m_port) == pros::c::v5_device_e_t::E_DEVICE_MOTOR; }

Angle Motor::getAngle() {
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

int Motor::setAngle(Angle angle) {
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

Angle Motor::getOffset() const { return m_offset; }

int Motor::setOffset(Angle offset) {
    m_offset = offset;
    return 0;
}

MotorType Motor::getType() {
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

int Motor::isReversed() const {
    // technically this returns an int, but as long as you only pass 0 to the index its impossible for it to return an
    // error. This is because we keep track of whether the motor is reversed or not through the sign of its port
    return m_port < 0;
}

int Motor::setReversed(bool reversed) {
    // technically this returns an int, but as long as you only pass 0 to the index its impossible for it to return an
    // error. This is because we keep track of whether the motor is reversed or not through the sign of its port
    if (reversed) {
        m_port = -abs(m_port);
    } else {
        m_port = abs(m_port);
    }
    return 0;
}

int Motor::getPort() const { return m_port; }

NEW_METRIC_PREFIXES(Current, amp) //temporary until added to units

Current Motor::motorToBatt(Current current) const {
    return from_amp(exp(current.convert(amp) + 4.324) / 0.925); 
}

Current Motor::battToMotor(Current current) const {
    return from_amp(0.925 * log(current.convert(amp)) - 4.324);
}

Current Motor::getCurrentLimit() const {
    const Current maxCurrent = 12.8_amp;
    const Current maxLimit = 2.5_amp;

    const Current thisLimit = from_mamp(pros::c::motor_get_current_limit(m_port));
    if (thisLimit.internal() == INT32_MAX) return from_amp(INFINITY); // error checking

    //gets the ports of all the motors
    std::vector<int> ports;
    Power power = 0_watt;
        for (int i = 0; i < 20; i++) {
            Motor m(i,200_rpm);
            if (!m.isConnected()) continue;
            else {
                ports.push_back(i);
                if(m.getType()==MotorType::V5) power += 11_watt;
                else power += 5.5_watt;
            }
        }
    //default behaviour if <= 88 watt
    if (power <= 88_watt) {
        //round thisLimit to 2dp
        return from_amp(std::round(to_amp(thisLimit)*100)/100);
    }
        
    //limit based on number of motors
    Current defaultLimit = battToMotor(maxCurrent/ports.size());
    if (defaultLimit > maxLimit) defaultLimit = maxLimit;
    
    //sum the limited motors
    Current totalLimit = 0_amp;
    int numLimited = 0;
    for (int i = 0; i < ports.size(); i++) {
        const Current limit = from_mamp(pros::c::motor_get_current_limit(ports[i]));
        if (limit.internal() == INT32_MAX) return from_amp(INFINITY); // error checking

        if (limit < defaultLimit){
            totalLimit += motorToBatt(limit);
            numLimited++;
        }
    }
    //adjust limit based on set limits
    const Current battCurrentPerMotor = (maxCurrent - totalLimit)/(ports.size()-numLimited);
    defaultLimit = battToMotor(battCurrentPerMotor);
    if (defaultLimit > maxLimit) defaultLimit = maxLimit;

    //check if set limit should apply
    if(thisLimit > defaultLimit){
        //round defaultLimit to 2dp
        return from_amp(std::round(to_amp(defaultLimit)*100)/100);
    }
    //round thisLimit to 2dp
    return from_amp(std::round(to_amp(thisLimit)*100)/100);
}

int Motor::setCurrentLimit(Current limit) { return pros::c::motor_set_current_limit(m_port, to_amp(limit) * 1000); }

Temperature Motor::getTemperature() const {
    const Temperature result = units::from_celsius(pros::c::motor_get_temperature(m_port));
    if (result.internal() == INFINITY) return result; // error checking
    return result;
}

// Always returns 0 because the velocity setter is not dependent on hardware and should never fail
int Motor::setOutputVelocity(AngularVelocity outputVelocity) {
    Angle angle = getAngle();
    m_outputVelocity = outputVelocity;
    setAngle(angle);
    return 0;
}
} // namespace lemlib