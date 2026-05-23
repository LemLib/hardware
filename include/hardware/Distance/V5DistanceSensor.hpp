#pragma once

#include "hardware/Distance/DistanceSensor.hpp"
#include "hardware/Port.hpp"
#include "pros/distance.hpp"

namespace lemlib {
/**
 * @brief Distance sensor implementation for the V5 Distance sensor
 *
 */
class V5DistanceSensor : public DistanceSensor {
    public:
        /**
         * @brief Construct a new V5 Distance Sensor
         *
         * @param port the unsigned port of the distance sensor.
         *
         * @b Example:
         * @code {.cpp}
         * void initialize() {
         *     // distance sensor on port 1
         *     lemlib::V5DistanceSensor distance(1);
         * }
         * @endcode
         */
        V5DistanceSensor(SmartPort port);
        /**
         * @brief V5DistanceSensor copy constructor
         *
         * Because pros::Mutex does not have a copy constructor, an explicit
         * copy constructor for the V5DistanceSensor is necessary
         *
         * @param other the V5DistanceSensor to copy
         */
        V5DistanceSensor(const V5DistanceSensor& other);
        /**
         * @brief Create a new V5 Distance Sensor
         *
         * @param encoder the pros::Distance object to use
         *
         * @b Example:
         * @code {.cpp}
         * void initialize() {
         *     lemlib::V5DistanceSensor distance = lemlib::V5DistanceSensor::from_pros_dist(pros::Distance(1));
         * }
         * @endcode
         */
        static V5DistanceSensor from_pros_dist(pros::Distance distanceSensor);
        /**
         * @brief whether the V5 Distance Sensor is connected
         *
         * @return 0 if its not connected
         * @return 1 if it is connected
         *
         * @b Example:
         * @code {.cpp}
         * void initialize() {
         *     lemlib::V5DistanceSensor distance = lemlib::V5DistanceSensor(1);
         *     const int result = distance.isConnected();
         *     if (result == 1) {
         *         std::cout << "Distance sensor is connected!" << std::endl;
         *     } else if (result == 0) {
         *         std::cout << "Distance sensor is not connected!" << std::endl;
         *     } else {
         *         std::cout << "Error checking if distance sensor is connected!" << std::endl;
         *     }
         * }
         * @endcode
         */
        int32_t isConnected() const override;
        /**
         * @brief Get the distance measured by the V5 Distance Sensor
         *
         * This function uses the following values of errno when an error state is reached:
         *
         * ENXIO: the port is not within the range of valid ports (1-21)
         * ENODEV: the port cannot be configured as an V5 Distance sensor
         *
         * @return Length the relative distance measured by the distance sensor
         * @return INFINITY if there is an error, setting errno
         *
         * @b Example:
         * @code {.cpp}
         * void initialize() {
         *     lemlib::V5DistanceSensor distanceSensor = lemlib::V5DistanceSensor(1);
         *     const Length distance = distanceSensor.getDistance();
         *     if (angle == INFINITY) {
         *         std::cout << "Error getting distance!" << std::endl;
         *     } else {
         *         std::cout << "Distance: " << distance.convert(in) << std::endl;
         *     }
         * }
         * @endcode
         */
        Length getDistance() const override;
        /**
         * @brief Set the offset of the distance sensor
         *
         * This function sets the offset of the distance sensor. A positive offset increases distance readings while a
         * negative offset decreases them. This function is non-blocking.
         *
         * @param distance the length to set the offset to
         * @return 0 on success
         * @return INT_MAX on failure, setting errno
         *
         * @b Example:
         * @code {.cpp}
         * void initialize() {
         *     lemlib::V5DistanceSensor distanceSensor(1);
         *     if (distanceSensor->setOffset(1_in) == 0) {
         *         std::cout << "Offset set!" << std::endl;
         *         // With an object touching the surface of the distance sensor
         *         std::cout << "Offset distance: " << distanceSensor->getDistance().convert(in) << std::endl; //
         * outputs 1 inch } else { std::cout << "Error setting offset!" << std::endl;
         *     }
         * }
         * @endcode
         */
        int32_t setOffset(Length distance) override;
    private:
        mutable pros::Mutex m_mutex;
        Length m_offset = 0_in;
        int m_port;
};
} // namespace lemlib