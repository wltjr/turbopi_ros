/** Copyright 2024 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#include <stdlib.h>
#include <math.h>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"

#include "joint.hpp"

namespace turbopi
{
	Joint::Joint()
	{
	}

    Joint::Joint(uint8_t id, uint8_t type, I2C &i2c)
    {
        this->id_ = id;
        this->type_ = type;
        this->i2c_ = &i2c;
    }

	Joint::~Joint()
	{
	}

	void Joint::setType(uint8_t type)
	{
		this->type_ = type;
	}

	uint8_t Joint::getId()
	{
		return this->id_;
	}

	double Joint::readAngle()
	{
		if (type_ == TYPE_MOTOR)
		{
			int8_t position;

			uint8_t result = i2c_->readBytes(id_ - 1 + MOTOR_ADDRESS, 1, position);
			if (result == 1)
			{
				return position;
			}
			else
			{
				RCLCPP_ERROR(rclcpp::get_logger(CLASS_NAME),
							 "I2C Read Error during joint %s position read.",
                             name.c_str());
				return 0;
			}
		}
		else if (type_ == TYPE_SERVO)
		{
			return _previousEffort;
		}
		else
		{
			return 0;
		}
	}

	void Joint::actuate(double effort, uint8_t /*duration = 1*/)
	{
		uint8_t data[2];

		if (type_ == TYPE_MOTOR)
		{
			const int LOW = 70;
			const int HIGH = 100;
			const int THRESHOLD = 30;

			if (effort > 100.0)
				effort = 100.0;
			else if (effort < -100.0)
				effort = -100.0;

			uint8_t speed = effort;

			// stick to 2 speeds low/high, motors need min ~35 to activate.
			if(effort > 0)
			{
				if(effort < THRESHOLD)
					speed = LOW;
				else
					speed = HIGH;
			}
			else if(effort < 0)
			{
				if(effort > -THRESHOLD)
					speed = -LOW;
				else
					speed = -HIGH;
			}

			// invert speeds for right side
			if(id_ & 1 && speed != 0)
				speed = -speed;

			data[0] = id_ - 1;
			data[1] = speed;

			uint8_t result = i2c_->writeData(MOTOR_ADDRESS, data);
			RCLCPP_INFO(rclcpp::get_logger(CLASS_NAME),
			            "write: %i; effort: %f; motor: %i, speed: %i",
						result, effort, data[0], data[1]);
		}
		else if (type_ == TYPE_SERVO)
		{
			if (effort != _previousEffort)
			{
                uint8_t angle = effort * 90 + 90;
                uint8_t pulse = ((200 * angle) / 9) + 500;
                uint8_t pulse_data[4];

                if (angle > 180)
                    angle = (uint8_t)180;
                else if (angle > max_)
                    angle = max_;

                data[0] = id_ - 5;
                data[1] = angle;

                pulse_data[0] = 1;
                pulse_data[1] = (uint8_t)10;
                pulse_data[2] = data[0];
                pulse_data[3] = pulse;

				uint8_t result = i2c_->writeData(CAMERA_ADDRESS, data);
				RCLCPP_INFO(rclcpp::get_logger(CLASS_NAME),
				            "write: %i; effort: %f; joint %s max %i servo: %i angle: %i°",
							result, effort, name.c_str(), max_, data[0], data[1]);

				result = i2c_->writeData(SERVO_ADDRESS_CMD, pulse_data);
				RCLCPP_INFO(rclcpp::get_logger(CLASS_NAME),
				            "write: %i; effort: %f; servo: %i time: %i pulse %i",
							result, effort, pulse_data[2], pulse_data[1], pulse_data[3]);
			}
		}

		_previousEffort = effort;
	}

	void Joint::setLimits(uint8_t min, uint8_t max)
	{
		this->min_ = min;
		this->max_ = max;
	}

	double Joint::getPreviousEffort()
	{
		return this->_previousEffort;
	}

	int Joint::getType()
	{
		return type_;
	}
}
