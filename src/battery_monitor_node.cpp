/** Copyright 2026 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#include <memory>

#include "battery_monitor_node.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace turbopi
{

    BatteryMonitorNode::BatteryMonitorNode(const rclcpp::NodeOptions &options)
        : Node("battery_monitor", options)
    {
        // subscribe to /battery topic topic for battery messages
        subscription_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
            "battery", rclcpp::QoS(10), std::bind(&BatteryMonitorNode::callbackBatterySubscriber, this, _1));
    }

    void BatteryMonitorNode::callbackBatterySubscriber(const sensor_msgs::msg::BatteryState::SharedPtr msg)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("BatteryMonitorNode"), "Battery voltage %f", msg->voltage);

        if(msg->voltage == 0)
        {
            RCLCPP_INFO(rclcpp::get_logger("BatteryMonitorNode"), "No reading from battery node");
            return;
        }

        if(msg->voltage > 3 && msg->voltage < 4.3)
        {
            RCLCPP_INFO(rclcpp::get_logger("BatteryMonitorNode"),
                        "Battery low voltage %f, shutting down",
                        msg->voltage);
            std::system("sudo init 0");
        }
        // on AC
        else if(msg->voltage < 6 && last > 6)
        {
            RCLCPP_INFO(rclcpp::get_logger("BatteryMonitorNode"),
                        "Switch from Battery to AC voltage %f",
                        msg->voltage);
        }
        // on Battery
        else if(msg->voltage > 6 && last < 6 )
        {
            RCLCPP_INFO(rclcpp::get_logger("BatteryMonitorNode"),
                        "Switch from AC to Battery voltage %f",
                        msg->voltage);
        }
        else if(msg->voltage > 8.5)
        {
            RCLCPP_INFO(rclcpp::get_logger("BatteryMonitorNode"),
                        "Battery high voltage %f, shutting down",
                        msg->voltage);
            std::system("sudo init 0");
        }
        // print interesting values for improvement
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("BatteryMonitorNode"), "Battery voltage %f", msg->voltage);
            return; // avoid garbage last reading
        }

        last = msg->voltage;
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

    rclcpp::spin(std::make_unique<turbopi::BatteryMonitorNode>(rclcpp::NodeOptions()));

    rclcpp::shutdown();

    return 0;
}
