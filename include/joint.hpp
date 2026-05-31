/** Copyright 2024 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *
 *  Updated for Pi5 / ROS Robot Controller (STM32) board.
 *  Motor and servo control now go through the Board serial class
 *  instead of direct I2C register writes.
 */

#ifndef TURBOPI__JOINT_H
#define TURBOPI__JOINT_H

#include <cmath>
#include <limits>
#include <sstream>

#include "board.hpp"

// Joint types
const uint8_t TYPE_NONE  = 255;
const uint8_t TYPE_MOTOR = 0;
const uint8_t TYPE_SERVO = 1;

extern char const* const CLASS_NAME;

namespace turbopi
{
    /**
     * @brief Class to hardware interface with and represent motors, servos,
     *        and other joints connected to the robot via the Board (STM32).
     */
	class Joint
	{
		public:
			double sensorResolution = 1024;
			std::string name;

            /**
             * @brief Construct a new Joint object, empty/unused
             */
			Joint();

            /**
             * @brief Construct a new Joint object, primary means to create a
             *        new joint with a read-only id, type, and board reference.
             *
             * @param id    1-based joint id (matches motor/servo id on the board)
             * @param type  type of joint: TYPE_MOTOR or TYPE_SERVO
             * @param board reference to the Board serial interface
             */
			Joint(uint8_t id, uint8_t type, Board &board);

            /**
             * @brief Destroy the Joint object, empty/unused
             */
			~Joint();

            /**
             * @brief Actuate the joint
             *
             * @param effort    the effort of the actuation (velocity for motors,
             *                  position [-1..1] for servos)
             * @param duration  duration hint (used for servo moves, seconds)
             */
			void actuate(double effort, uint8_t duration);

            /**
             * @brief Get the type of joint
             *
             * @return int type of joint from DEFINES; motor, servo, etc
             */
			int getType();

            /**
             * @brief Get the joint id
             *
             * @return uint8_t internal joint id (1-based)
             */
			uint8_t getId();

	        double getPreviousEffort();

            /**
             * @brief Get the joint's current value
             *        For motors: returns last commanded velocity (odometry not
             *        available via serial without encoder feedback).
             *
             * @return double joint value
             */
			double getValue();

            /**
             * @brief Set the type of joint
             *
             * @param type type of joint from DEFINES
             */
			void setType(uint8_t type);

            /**
             * @brief Set the joint limits (used for servo range clamping)
             *
             * @param min   the minimum joint value (degrees)
             * @param max   the maximum joint value (degrees)
             */
			void setLimits(uint8_t min, uint8_t max);

		private:
            Board  *board_;
			uint8_t id_   = 0;
			uint8_t max_  = 75;
			uint8_t min_  = 0;
			uint8_t type_ = 0;
			double  _previousEffort = std::numeric_limits<double>::quiet_NaN();
	};
}

#endif
