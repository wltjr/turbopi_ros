/** Copyright 2024 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *
 *  Camera control updated to use JointTrajectoryController (JTC) instead of
 *  the old JointGroupPositionController.  The right joystick axes increment/
 *  decrement an accumulated camera_pan_ / camera_tilt_ position [-1..1] and
 *  publish a JointTrajectory message to position_controllers/joint_trajectory.
 *
 *  Mecanum strafe: D-pad X → twist.linear.y (sideways movement, no rotation).
 *  Honk:          SQUARE button → publishes a short beep to /buzzer.
 */

#include "teleop_turbopi.hpp"

using std::placeholders::_1;

namespace teleop_turbopi
{
    TurboPi::TurboPi(const rclcpp::NodeOptions &options)
        : Node("teleop_turbopi", options)
    {
        // publish cmd_vel for wheel drive
        publisher_cmd_vel_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);

        // publish JointTrajectory for the camera JointTrajectoryController
        publisher_pos_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "position_controllers/joint_trajectory", 10);

        // publish to /buzzer topic: Float32MultiArray [freq, on_time, off_time, repeat]
        publisher_buzzer_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/buzzer", 10);

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

        // ── Honk (SQUARE button) ───────────────────────────────────────────────
        // Trigger on press edge only (rising edge: was 0, now 1)
        bool square_now = joy_msg->buttons[static_cast<int>(TurboPi::buttons::SQUARE)] != 0;
        if (square_now && !square_was_pressed_)
        {
            // Short honk: 1900 Hz, 0.1 s on, 0.05 s off, 2 beeps
            auto buzzer_msg = std::make_unique<std_msgs::msg::Float32MultiArray>();
            buzzer_msg->data = {1900.0f, 0.15f, 0.05f, 2};
            publisher_buzzer_->publish(std::move(buzzer_msg));
        }
        square_was_pressed_ = square_now;

        // ── Wheel drive (cmd_vel) ──────────────────────────────────────────────
        auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        cmd_vel_msg->header.stamp = get_clock()->now();
        cmd_vel_msg->header.frame_id = "";
        cmd_vel_msg->twist.linear.x  = LINEAR_SCALE  * joy_msg->axes[std::to_underlying(TurboPi::axes::LEFT_JOY_Y)];
        cmd_vel_msg->twist.angular.z = ANGULAR_SCALE * joy_msg->axes[std::to_underlying(TurboPi::axes::LEFT_JOY_X)];
        // D-pad X: lateral strafe (mecanum only – diff drive ignores linear.y)
        // DPAD_X: +1.0 = left, -1.0 = right (standard joy_linux convention)
        cmd_vel_msg->twist.linear.y  = STRAFE_SCALE  * joy_msg->axes[std::to_underlying(TurboPi::axes::DPAD_X)];
        publisher_cmd_vel_->publish(std::move(cmd_vel_msg));

        // ── Camera pan/tilt (JointTrajectory) ─────────────────────────────────
        // Right joystick Y axis → pan (camera_joint)
        // Right joystick X axis → tilt (camera_frame_joint)
        // Each callback increments the accumulated position by axis_value * CAMERA_STEP.
        // This gives velocity-like feel: hold stick → camera moves continuously.
        double pan_axis  = joy_msg->axes[std::to_underlying(TurboPi::axes::RIGHT_JOY_Y)];
        double tilt_axis = joy_msg->axes[std::to_underlying(TurboPi::axes::RIGHT_JOY_X)];

        // Only publish when there is actual joystick input (dead-zone ~0.05)
        if (std::abs(pan_axis) > 0.05 || std::abs(tilt_axis) > 0.05)
        {
            camera_pan_  += pan_axis  * CAMERA_STEP;
            camera_tilt_ += tilt_axis * CAMERA_STEP;

            // Clamp to [-1..1]
            if (camera_pan_  >  1.0) camera_pan_  =  1.0;
            if (camera_pan_  < -1.0) camera_pan_  = -1.0;
            if (camera_tilt_ >  1.0) camera_tilt_ =  1.0;
            if (camera_tilt_ < -1.0) camera_tilt_ = -1.0;

            auto traj_msg = std::make_unique<trajectory_msgs::msg::JointTrajectory>();
            traj_msg->joint_names = {"camera_joint", "camera_frame_joint"};

            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = {camera_pan_, camera_tilt_};
            // Short duration so the servo responds quickly
            point.time_from_start = rclcpp::Duration::from_seconds(0.1);

            traj_msg->points.push_back(point);
            publisher_pos_->publish(std::move(traj_msg));
        }
    }
}
