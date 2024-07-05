/** Copyright 2024 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#ifndef TELEOP_MANUAL_JOY__TELEOP_MANUAL_JOY_HPP_
#define TELEOP_MANUAL_JOY__TELEOP_MANUAL_JOY_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_options.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace teleop_turbopi
{
    /**
     * @brief Class for joystick/gamepad remote control, maps control actions
     *        to commands and velocities
     */
    class TurboPi : public rclcpp::Node
    {
        public:

            /**
             * @brief DPad and Joystick axes on the DualShock 4 gamepad
             */
            enum class axes: int
            {
                LEFT_JOY_X,
                LEFT_JOY_Y,
                RIGHT_JOY_X,
                LEFT_TRIGGER,
                RIGHT_TRIGGER,
                RIGHT_JOY_Y,
                DPAD_X,
                DPAD_Y
            };

            /**
             * @brief Buttons on the DualShock 4 gamepad
             */
            enum class buttons: int
            {
                SQUARE,
                X,
                CIRCLE,
                TRIANGLE,
                LEFT_BUMPER,
                RIGHT_BUMPER,
                LEFT_TRIGGER_ON,
                RIGHT_TRIGGER_ON,
                SHARE,
                OPTIONS,
                LEFT_JOY_CLICK,
                RIGHT_JOY_CLICK,
                PS,
                TOUCH_PAD
            };

            /**
             * @brief Construct a new Teleop TurboPi instance
             * 
             * @param options node options passed at startup
             */
            explicit TurboPi(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

            /**
             * @brief Destroy the Teleop TurboPi object
             * 
             */
            virtual ~TurboPi();

        private:
            // publishers - topics we publish commands to; cmd_vel and positions
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd_vel_;
            rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_pos_;

            // subscriber - the joy topic we listen to for joystick buttons
            rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;

            /**
             * @brief Callback function for subscription fired when messages
             *        on the joy topic are heard
             * 
             * @param msg the message data that was heard
             */
            void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);
    };
}

#endif
