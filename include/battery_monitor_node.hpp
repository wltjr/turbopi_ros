/** Copyright 2026 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#ifndef TURBOPI__BATTERYMONITOR__NODE_H
#define TURBOPI__BATTERYMONITOR__NODE_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_options.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace turbopi
{
    /**
     * @brief Class to Monitor the Battery Node
     */
	class BatteryMonitorNode : public rclcpp::Node
	{
		public:

            /**
             * @brief Construct a new BatteryMonitorNode object
             *
             * @param options node options passed at startup
             */
            explicit BatteryMonitorNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

            /**
             * @brief 
             * @brief Callback function for subscriber fired when messages
             *        on the /battery topic are received
             * @param msg 
             */
            void callbackBatterySubscriber(const sensor_msgs::msg::BatteryState::SharedPtr msg);

		private:
            // last voltage reading, 6 for neutrality AC/battery
            float last = 6;

            // subscriber - the /battery topic we listen to
            rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr subscription_;
	};
}

#endif
