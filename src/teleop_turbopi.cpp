/** Copyright 2024 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#include "teleop_turbopi.hpp"

using std::placeholders::_1;

namespace teleop_turbopi
{
    TurboPi::TurboPi(const rclcpp::NodeOptions &options)
        : Node("teleop_turbopi", options)
    {
        // publish to controller specific topics, may combine into unified
        publisher_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        publisher_pos_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("position_controllers/commands", 10);

        // subscribe to /joy topic for joystick messages
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", rclcpp::QoS(10), std::bind(&TurboPi::joyCallback, this, _1));
    }

    TurboPi::~TurboPi() = default;

    void TurboPi::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
        // system shutdown
        if (joy_msg->buttons[static_cast<int>(TurboPi::buttons::SHARE)])
        {
            std::system("sudo init 0");
        }

        // Initializes with zeros by default.
        auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();

        // set wheel velocity
        cmd_vel_msg->linear.x = joy_msg->axes[std::to_underlying(TurboPi::axes::LEFT_JOY_Y)];
        cmd_vel_msg->angular.z = joy_msg->axes[std::to_underlying(TurboPi::axes::LEFT_JOY_X)];

        // publish velocities
        publisher_cmd_vel_->publish(std::move(cmd_vel_msg));

        // Initializes with zeros by default.
        auto pos_msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
        pos_msg->data.resize(2);

        // camera left/right
        if (joy_msg->axes[static_cast<int>(TurboPi::axes::RIGHT_JOY_Y)])
        {
            pos_msg->data[0] = joy_msg->axes[std::to_underlying(TurboPi::axes::RIGHT_JOY_Y)];
        }
        else
        {
            // camera stop
            pos_msg->data[0] = 0;
        }

        // camera up/down
        if (joy_msg->axes[static_cast<int>(TurboPi::axes::RIGHT_JOY_X)])
        {
            pos_msg->data[1] = joy_msg->axes[std::to_underlying(TurboPi::axes::RIGHT_JOY_X)];
        }
        else
        {
            // camera stop
            pos_msg->data[1] = 0;
        }

        // publish positions
        publisher_pos_->publish(std::move(pos_msg));
    }
}
