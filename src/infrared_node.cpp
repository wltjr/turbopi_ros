/** Copyright 2024 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#include <memory>

#include "infrared.hpp"
#include "infrared_node.hpp"

using namespace std::chrono_literals;

namespace turbopi
{

    InfraredNode::InfraredNode(const rclcpp::NodeOptions &options)
        : Node("infrared", options)
    {
        static auto infrared = Infrared(1, INFRARED_ADDRESS);
        infrared_ = &infrared;

        // publish to /infrared topic
        publisher_infrared_ =
            this->create_publisher<std_msgs::msg::UInt8MultiArray>("infrared",
                rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

        timer_infrared_ = 
            this->create_wall_timer(500ms, std::bind(&InfraredNode::callbackInfraredPublisher, this));
    }

    void InfraredNode::callbackInfraredPublisher()
    {
        // Initializes with zeros by default.
        auto msg = std::make_unique<std_msgs::msg::UInt8MultiArray>();
        msg->data.resize(4);

        auto values = infrared_->getValues();
        msg->data.assign(values.begin(), values.end());

        publisher_infrared_->publish(std::move(msg));
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

    rclcpp::spin(std::make_unique<turbopi::InfraredNode>(rclcpp::NodeOptions()));

    rclcpp::shutdown();

    return 0;
}
