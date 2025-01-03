/** Copyright 2024 William L Thomson Jr <w@wltjr.com>
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#include <stdlib.h>
#include <math.h>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"

#include "joint.hpp"

namespace turbopi
{
    Joint::Joint() = default;

    Joint::Joint(uint8_t id, uint8_t type, I2C &i2c) : i2c_(&i2c), id_(id), type_(type)
    {
    }

    Joint::~Joint() = default;

    void Joint::setType(uint8_t type)
    {
        this->type_ = type;
    }

    uint8_t Joint::getId()
    {
        return this->id_;
    }

    double Joint::getValue()
    {
        if (type_ == TYPE_MOTOR)
        {
            std::vector<uint8_t> data(1);

            if (i2c_->readBytes(id_ - 1 + MOTOR_ADDRESS, 1, data))
            {
                return data[0];
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger(CLASS_NAME),
                             "I2C Read Error during joint %s position read.",
                             name.c_str());
                return 0;
            }
        }
        else if (type_ == TYPE_SERVO)
        {
            return _previousEffort;
        }
        else
        {
            return 0;
        }
    }

    void Joint::actuate(double effort, uint8_t /*duration = 1*/)
    {
        std::array<uint8_t, 2> data;

        if (type_ == TYPE_MOTOR)
        {
            int8_t speed = 0;

            if (floor(effort) != 0)
            {
                const int8_t LOW = 50;

                speed = ceil(effort) / 31 * 100;

                if (speed > 100)
                    speed = 100;
                else if (speed < -100)
                    speed = -100;
                else if (speed > 0 && speed < LOW)
                    speed = LOW;
                else if (speed < 0 && speed > -LOW)
                    speed = -LOW;

                // invert speeds for right side
                if (id_ & 1)
                    speed = -speed;
            }

            data[0] = id_ - 1;
            data[1] = speed;

            uint8_t result = i2c_->writeData(MOTOR_ADDRESS, data);
            RCLCPP_DEBUG(rclcpp::get_logger(CLASS_NAME),
                         "write: %i; effort: %f; motor: %i, speed: %i",
                         result, effort, data[0], data[1]);
        }
        else if (type_ == TYPE_SERVO)
        {
            if (effort != _previousEffort)
            {
                uint8_t angle = effort * 90 + 90;
                uint8_t pulse = ((200 * angle) / 9) + 500;
                std::array<uint8_t, 2> pulse_data;

                if (angle > 180)
                    angle = (uint8_t)180;
                else if (angle > max_)
                    angle = max_;

                data[0] = id_ - 5;
                data[1] = angle;

                pulse_data[0] = 1;
                pulse_data[1] = (uint8_t)10;
                pulse_data[2] = data[0];
                pulse_data[3] = pulse;

                uint8_t result = i2c_->writeData(CAMERA_ADDRESS, data);
                RCLCPP_DEBUG(rclcpp::get_logger(CLASS_NAME),
                             "write: %i; effort: %f; joint %s max %i servo: %i angle: %i°",
                             result, effort, name.c_str(), max_, data[0], data[1]);

                result = i2c_->writeData(SERVO_ADDRESS_CMD, pulse_data);
                RCLCPP_DEBUG(rclcpp::get_logger(CLASS_NAME),
                             "write: %i; effort: %f; servo: %i time: %i pulse %i",
                             result, effort, pulse_data[2], pulse_data[1], pulse_data[3]);
            }
        }

        _previousEffort = effort;
    }

    void Joint::setLimits(uint8_t min, uint8_t max)
    {
        this->min_ = min;
        this->max_ = max;
    }

    double Joint::getPreviousEffort()
    {
        return this->_previousEffort;
    }

    int Joint::getType()
    {
        return type_;
    }
}
