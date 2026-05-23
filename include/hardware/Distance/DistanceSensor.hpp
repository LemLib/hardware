#pragma once

#include "hardware/Device.hpp"
#include "units/units.hpp"

namespace lemlib {
/**
 * @brief abstract class for distance sensors
 *
 * An distance sensor is a device that measures the distance to an external object. This class is abstract, and must be
 * implemented by a subclass.
 */
class DistanceSensor : public Device {
    public:
        /**
         * @brief Get the distance measured by the distance sensor
         *
         * The distance measured by the distance sensor is the distance from the sensor to the nearest object.
         *
         * @return Length the distance measured by the encoder
         * @return INFINITY if there is an error, setting errno
         *
         * @b Example:
         * @code {.cpp}
         * void initialize() {
         *     DistanceSensor* distanceSensor;
         *     const Length distance = distanceSensor->getDistance();
         *     if (distance == INFINITY) {
         *         std::cout << "Error getting distance!" << std::endl;
         *     } else {
         *         std::cout << "Distance: " << distance.convert(in) << std::endl;
         *     }
         * }
         * @endcode
         */
        virtual Length getDistance() const = 0;
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
         *     DistanceSensor* distanceSensor;
         *     if (distanceSensor->setOffset(1_in) == 0) {
         *         std::cout << "Offset set!" << std::endl;
         *         // With an object touching the surface of the distance sensor
         *         std::cout << "Offset distance: " << distanceSensor->getDistance().convert(in) << std::endl; //
         * outputs 1 inch } else { std::cout << "Error setting offset!" << std::endl;
         *     }
         * }
         * @endcode
         */
        virtual int32_t setOffset(Length distance) = 0;
        virtual ~DistanceSensor() = default;
};
} // namespace lemlib