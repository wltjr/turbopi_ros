/** Copyright 2024 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#include <memory>

#include "sonar.hpp"
#include "sonar_node.hpp"

using namespace std::chrono_literals;

namespace turbopi
{

    SonarNode::SonarNode(const rclcpp::NodeOptions &options)
        : Node("sonar", options)
    {
        static auto sonar = Sonar(1, SONAR_ADDRESS);
        sonar.setRGBMode(0);
        sonar.setPixelColor(0,0); // #000000 - black/off
        sonar.setPixelColor(1,0);
        sonar_ = &sonar;

        // publish to /sonar topic
        publisher_sonar_ =
            this->create_publisher<sensor_msgs::msg::Range>("sonar",
                rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

        timer_sonar_ = 
            this->create_wall_timer(500ms, std::bind(&SonarNode::callbackSonarPublisher, this));
    }

    void SonarNode::callbackSonarPublisher()
    {
        // Initializes with zeros by default.
        auto msg = std::make_unique<sensor_msgs::msg::Range>();

        msg->header.stamp = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
        msg->header.frame_id = "sonar";
        msg->radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
        msg->field_of_view = 0.2617994; // 15°
        msg->min_range = 0.02;
        msg->max_range = 4;
        msg->range = sonar_->getDistance();

        publisher_sonar_->publish(std::move(msg));
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

    rclcpp::spin(std::make_unique<turbopi::SonarNode>(rclcpp::NodeOptions()));

    rclcpp::shutdown();

    return 0;
}
