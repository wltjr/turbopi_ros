/** Copyright 2025 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#include <memory>

#include "battery.hpp"
#include "battery_node.hpp"

using namespace std::chrono_literals;

namespace turbopi
{

    BatteryNode::BatteryNode(const rclcpp::NodeOptions &options)
        : Node("battery", options)
    {
        static auto battery = Battery(1, BATTERY_ADDRESS);
        battery_ = &battery;

        // publish to /battery topic
        publisher_battery_ =
            this->create_publisher<sensor_msgs::msg::BatteryState>("battery",
                rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

        timer_battery_ = 
            this->create_wall_timer(2000ms, std::bind(&BatteryNode::callbackBatteryPublisher, this));
    }

    void BatteryNode::callbackBatteryPublisher()
    {
        // Initializes with zeros by default.
        auto msg = std::make_unique<sensor_msgs::msg::BatteryState>();

        msg->header.stamp = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
        msg->header.frame_id = "battery";
        msg->voltage = battery_->getVoltage();

        publisher_battery_->publish(std::move(msg));
    }
}

/**
 * @brief Node executable wrapper for the shared object
 * 
 * @param argc the count of arguments passed to the program on start
 * @param argv the argument values passed to the program on start
 * @return int program return code
 */
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_unique<turbopi::BatteryNode>(rclcpp::NodeOptions()));

    rclcpp::shutdown();

    return 0;
}
