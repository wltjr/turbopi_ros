/** Copyright 2024 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#ifndef TURBOPI__INFRARED__NODE_H
#define TURBOPI__INFRARED__NODE_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_options.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace turbopi
{
    /**
     * @brief Class to interface with the Infrared hardware class
     */
	class InfraredNode : public rclcpp::Node
	{
		public:

            /**
             * @brief Construct a new InfraredNode object
             *
             * @param options node options passed at startup
             */
            explicit InfraredNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

            /**
             * @brief Callback function for publisher fired when messages
             *        on the /infrared topic are published
             */
            void callbackInfraredPublisher();

		private:
            Infrared *infrared_;

            // publisher - /infrared topic we publish merged maps to
            rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_infrared_;

            // timer
            rclcpp::TimerBase::SharedPtr timer_infrared_;

	};
}

#endif
