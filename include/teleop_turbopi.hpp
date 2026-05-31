/** Copyright 2024 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#ifndef TELEOP_MANUAL_JOY__TELEOP_MANUAL_JOY_HPP_
#define TELEOP_MANUAL_JOY__TELEOP_MANUAL_JOY_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_options.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
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
             *
             * Actual DS4 axis order from joy_linux_node (confirmed via ros2 topic echo /joy):
             *   0 = LEFT_JOY_X  (left/right)
             *   1 = LEFT_JOY_Y  (up/down)
             *   2 = LEFT_TRIGGER   (1.0=released, -1.0=fully pressed)
             *   3 = RIGHT_JOY_X (left/right)
             *   4 = RIGHT_JOY_Y (up/down)
             *   5 = RIGHT_TRIGGER  (1.0=released, -1.0=fully pressed)
             *   6 = DPAD_X
             *   7 = DPAD_Y
             */
            enum class axes: int
            {
                LEFT_JOY_X,
                LEFT_JOY_Y,
                LEFT_TRIGGER,
                RIGHT_JOY_X,
                RIGHT_JOY_Y,
                RIGHT_TRIGGER,
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
            // publishers - topics we publish commands to; cmd_vel, positions, buzzer
            rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_cmd_vel_;
            rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_pos_;
            rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_buzzer_;

            // subscriber - the joy topic we listen to for joystick buttons
            rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;

            // Accumulated camera position state [-1..1] for pan and tilt.
            // Joystick axes increment/decrement these values each callback.
            double camera_pan_  = 0.0;
            double camera_tilt_ = 0.0;

            // Debounce flag: track previous SQUARE button state to trigger honk
            // on press edge only (not held).
            bool square_was_pressed_ = false;

            // Step size per joystick callback (joystick axis value [-1..1] × step)
            static constexpr double CAMERA_STEP = 0.05;

            // Speed scale factors applied to joystick axes before publishing cmd_vel.
            // Reduce ANGULAR_SCALE to slow down in-place rotation (helps slam_toolbox
            // scan matching keep up during turns).
            static constexpr double LINEAR_SCALE  = 1.0;  // forward/back  (m/s at full stick)
            static constexpr double ANGULAR_SCALE = 0.5;  // rotation      (rad/s at full stick)
            static constexpr double STRAFE_SCALE  = 0.5;  // lateral strafe (m/s at full d-pad)

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
