/** Copyright 2024 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#ifndef TURBOPI__JOINT_H
#define TURBOPI__JOINT_H

#include <sstream>

#include "i2c.hpp"

const uint8_t BASE_SLAVE_ADDRESS = 0x7A;
const uint8_t CAMERA_ADDRESS = 21;
const uint8_t MOTOR_ADDRESS = 31;
const uint8_t SERVO_ADDRESS_CMD = 40;

const uint8_t TYPE_NONE = -1;
const uint8_t TYPE_MOTOR = 0;
const uint8_t TYPE_SERVO = 1;

extern char const* const CLASS_NAME;

namespace turbopi
{
    /**
     * @brief Class to hardware interface with and represent motors, servos,
     *        and other joints connected to the robot
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
             *        new joint with a read only id, type, and i2c device
             * 
             * @param id internal joint id, read only after creation
             * @param type type of joint from DEFINES; motor, servo, etc
             * @param i2c  i2c device for the joint
             */
			Joint(uint8_t id, uint8_t type, I2C &i2c);

            /**
             * @brief Destroy the Joint object, empty/unused
             */
			~Joint();

            /**
             * @brief Actuate the joint
             * 
             * @param effort    the effort of the actuation, speed, position, etc
             * @param duration  the duration of the effort
             */
			void actuate(double effort, uint8_t duration);

            /**
             * @brief Get the type of joint
             * 
             * @return int type of joint from DEFINES; motor, servo, etc
             */
			int getType();

            /**
             * @brief Get the type of joint
             * 
             * @return uint8_t internal joint id
             */
			uint8_t getId();

	        double getPreviousEffort();

            /**
             * @brief Get the joints curent value
             * 
             * @return double internal joint id
             */
			double getValue();

            /**
             * @brief Set the type of joint
             * 
             * @param type type of joint from DEFINES; motor, servo, etc
             */
			void setType(uint8_t type);

            /**
             * @brief Set the joint limits, minimum and maximum values
             * 
             * @param min   the minimum joint value
             * @param max   the maximum joint value
             */
			void setLimits(uint8_t min, uint8_t max);

		private:
            I2C *i2c_;
			uint8_t id_ = 0;
			uint8_t max_ = 75;
			uint8_t min_ = 0;
			uint8_t type_ = 0;
			double _previousEffort;
	};
}

#endif