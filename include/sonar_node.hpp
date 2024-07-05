/** Copyright 2024 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#ifndef TURBOPI__SONAR__NODE_H
#define TURBOPI__SONAR__NODE_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_options.hpp"
#include "sensor_msgs/msg/range.hpp"

namespace turbopi
{
    /**
     * @brief Class to interface with the Sonar hardware class
     */
	class SonarNode : public rclcpp::Node
	{
		public:

            /**
             * @brief Construct a new SonarNode object
             *
             * @param options node options passed at startup
             */
            explicit SonarNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

            /**
             * @brief Callback function for publisher fired when messages
             *        on the /sonar topic are published
             */
            void callbackSonarPublisher();

		private:
            Sonar *sonar_;

            // publisher - /sonar topic we publish merged maps to
            rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_sonar_;

            // timer
            rclcpp::TimerBase::SharedPtr timer_sonar_;

	};
}

#endif
