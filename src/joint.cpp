/** Copyright 2024 William L Thomson Jr <w@wltjr.com>
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *
 *  Updated for Pi5 / ROS Robot Controller (STM32) board.
 *  All motor and servo commands are now sent through the Board serial class
 *  (0xAA 0x55 packet protocol) instead of direct I2C register writes.
 *
 *  Motor inversion logic is preserved from the Pi4 version:
 *    - Odd motor ids (1, 3) are on the left side → speed is negated so that
 *      positive effort = forward on both sides, matching the Pi5 mecanum.py
 *      pattern: board.set_motor_duty([[1, -v1], [2, v2], [3, -v3], [4, v4]])
 *
 *  Servo PWM pulse calculation updated for the Pi5 STM32 board:
 *    pulse = (effort * 400) + 1500  µs  (range 1100-1900, center 1500)
 *  Reference: turbopi_ros2_pi5/src/driver/sdk/sdk/PWMServoControlDemo.py
 */

#include <stdlib.h>
#include <math.h>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"

#include "joint.hpp"

namespace turbopi
{
    Joint::Joint() = default;

    Joint::Joint(uint8_t id, uint8_t type, Board &board)
        : board_(&board), id_(id), type_(type)
    {
    }

    Joint::~Joint() = default;

    void Joint::setType(uint8_t type)
    {
        type_ = type;
    }

    uint8_t Joint::getId()
    {
        return id_;
    }

    double Joint::getValue()
    {
        // The Pi5 STM32 board does not expose encoder counts over the serial
        // protocol used here.  Return the last commanded value so that the
        // hardware interface can report plausible odometry.
        return _previousEffort;
    }

    void Joint::actuate(double effort, uint8_t /*duration*/)
    {
        if (type_ == TYPE_MOTOR)
        {
            float duty = 0.0f;

            if (floor(effort) != 0)
            {
                const float LOW = 50.0f;

                duty = static_cast<float>(ceil(effort) / 31.0 * 100.0);

                if (duty > 100.0f)
                    duty = 100.0f;
                else if (duty < -100.0f)
                    duty = -100.0f;
                else if (duty > 0.0f && duty < LOW)
                    duty = LOW;
                else if (duty < 0.0f && duty > -LOW)
                    duty = -LOW;

                // Invert left-side motors (ids 1 and 3 are odd).
                // Matches Pi5 mecanum.py: set_motor_duty([[1,-v1],[2,v2],[3,-v3],[4,v4]])
                if (id_ & 1u)
                    duty = -duty;
            }

            board_->setMotorDuty({{id_, duty}});

            RCLCPP_DEBUG(rclcpp::get_logger(CLASS_NAME),
                         "motor %u: effort=%.3f duty=%.1f", id_, effort, duty);
        }
        else if (type_ == TYPE_SERVO)
        {
            if (effort != _previousEffort)
            {
                // Map effort [-1..1] → pulse [1100..1900] µs, center 1500 µs.
                // Reference: turbopi_ros2_pi5 PWMServoControlDemo.py which uses
                // board.pwm_servo_set_position(t, [[id, pulse]]) with values
                // in the range 1100-1900 µs and center at 1500 µs.
                // Clamp effort to [-1..1] before computing pulse.
                double clamped = effort;
                if (clamped > 1.0)  clamped = 1.0;
                if (clamped < -1.0) clamped = -1.0;

                uint16_t pulse = static_cast<uint16_t>(clamped * 400.0 + 1500.0);

                // Safety clamp to valid hardware range
                if (pulse > 1900) pulse = 1900;
                if (pulse < 1100) pulse = 1100;

                // PWM servo id on the Pi5 board is 1-based (id_ 5→1, 6→2)
                uint8_t servo_id = id_ - 4u;

                // duration in seconds (0.1 s = fast move)
                board_->pwmServoSetPosition(0.1f, {{servo_id, pulse}});

                RCLCPP_INFO(rclcpp::get_logger(CLASS_NAME),
                             "servo %u (board id %u): effort=%.3f → pulse=%u µs",
                             id_, servo_id, effort, pulse);
            }
        }

        _previousEffort = effort;
    }

    void Joint::setLimits(uint8_t min, uint8_t max)
    {
        min_ = min;
        max_ = max;
    }

    double Joint::getPreviousEffort()
    {
        return _previousEffort;
    }

    int Joint::getType()
    {
        return type_;
    }
}
