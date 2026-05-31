/** Copyright 2025 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *
 *  Updated for Pi5: battery_node no longer opens /dev/rrc directly.
 *  It subscribes to /battery_voltage_mv (std_msgs/Int32, published by
 *  the ros2_control hardware interface) and re-publishes as
 *  sensor_msgs/BatteryState on /battery.
 */

#ifndef TURBOPI__BATTERY__NODE_H
#define TURBOPI__BATTERY__NODE_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_options.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/int32.hpp"

namespace turbopi
{
    /**
     * @brief Publishes sensor_msgs/BatteryState on /battery.
     *        Receives raw millivolt readings from /battery_voltage_mv
     *        (published by the ros2_control hardware interface, which is the
     *        sole owner of the /dev/rrc serial port).
     */
	class BatteryNode : public rclcpp::Node
	{
		public:

            /**
             * @brief Construct a new BatteryNode object
             *
             * @param options node options passed at startup
             */
            explicit BatteryNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

		private:
            // publisher – /battery topic (sensor_msgs/BatteryState)
            rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr publisher_battery_;

            // subscriber – /battery_voltage_mv topic (std_msgs/Int32, mV)
            rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_mv_;
	};
}

#endif
