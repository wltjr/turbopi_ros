/** Copyright 2025 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *
 *  Updated for Pi5 / ROS Robot Controller (STM32) board.
 *
 *  Battery voltage is now received via the /battery_voltage_mv topic published
 *  by the hardware interface (turbopi_battery_bridge node inside ros2_control).
 *  This avoids a second process opening /dev/rrc simultaneously, which caused
 *  interleaved UART reads and prevented battery packets from being parsed.
 *
 *  Flow:
 *    ros2_control (TurboPiSystemHardware::read)
 *      → publishes std_msgs/Int32 millivolts on /battery_voltage_mv
 *    battery_node (this file)
 *      → subscribes, converts mV → V, publishes sensor_msgs/BatteryState on /battery
 */

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/int32.hpp"

#include "battery_node.hpp"

using namespace std::chrono_literals;

namespace turbopi
{

    BatteryNode::BatteryNode(const rclcpp::NodeOptions &options)
        : Node("battery", options)
    {
        // Publish battery state on /battery (transient-local so late subscribers
        // receive the last known value immediately)
        publisher_battery_ =
            this->create_publisher<sensor_msgs::msg::BatteryState>(
                "battery",
                rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

        // Subscribe to raw millivolt readings from the hardware interface.
        // The hardware interface publishes every ~5 s; use transient_local so we
        // get the last value even if we start after the first publish.
        subscription_mv_ =
            this->create_subscription<std_msgs::msg::Int32>(
                "/battery_voltage_mv",
                rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
                [this](const std_msgs::msg::Int32::SharedPtr msg)
                {
                    auto battery_msg = std::make_unique<sensor_msgs::msg::BatteryState>();
                    battery_msg->header.stamp    = this->get_clock()->now();
                    battery_msg->header.frame_id = "battery";

                    int mv = msg->data;
                    if (mv > 0)
                    {
                        battery_msg->voltage = static_cast<float>(mv) / 1000.0f;
                        battery_msg->present = true;
                    }
                    else
                    {
                        battery_msg->voltage = 0.0f;
                        battery_msg->present = false;
                        RCLCPP_WARN(this->get_logger(),
                                    "Battery voltage not yet available from STM32");
                    }

                    publisher_battery_->publish(std::move(battery_msg));
                });

        RCLCPP_INFO(this->get_logger(),
                    "battery_node started – waiting for /battery_voltage_mv");
    }

}  // namespace turbopi

/**
 * @brief Node executable wrapper for the shared object
 */
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_unique<turbopi::BatteryNode>(rclcpp::NodeOptions()));

    rclcpp::shutdown();

    return 0;
}
