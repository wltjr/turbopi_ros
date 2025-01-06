/** Copyright 2025 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#ifndef TURBOPI__BATTERY__NODE_H
#define TURBOPI__BATTERY__NODE_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_options.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace turbopi
{
    /**
     * @brief Class to interface with the Battery hardware class
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

            /**
             * @brief Callback function for publisher fired when messages
             *        on the /battery topic are published
             */
            void callbackBatteryPublisher();

		private:
            Battery *battery_;

            // publisher - /battery topic we publish merged maps to
            rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr publisher_battery_;

            // timer
            rclcpp::TimerBase::SharedPtr timer_battery_;

	};
}

#endif
