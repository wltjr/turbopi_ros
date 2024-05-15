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

	double Joint::_filterAngle(double angle)
	{
		_angleReads = _angleReads + 1;

		// put value at front of array
		for (int i = _filterPrevious - 1; i > 0; i--)
		{
			_previousAngles[i] = _previousAngles[i - 1];
		}
		_previousAngles[0] = angle;

		int filterIterations = _filterPrevious;
		if (_angleReads < _filterPrevious)
		{
			filterIterations = _angleReads;
		}

		double angleSum = 0;
		for (int i = 0; i < filterIterations; i++)
		{
			angleSum = angleSum + _previousAngles[i];
		}

		double filterResult = angleSum / (filterIterations * 1.0);

		// RCLCPP_INFO(rclcpp::get_logger(CLASS_NAME),
		// 			"Joint: %s, Angle: %f, AngleSum: %f, FilterResult: %f, Iterations: %i",
		// 			name.c_str(), angle, angleSum, filterResult, filterIterations);

		return filterResult;
	}

	double Joint::readAngle()
	{
		if (type_ == TYPE_MOTOR)
		{
			int8_t position;
			const int TAU = M_PI + M_PI;

			uint8_t result = i2c_->readBytes(id_ - 1 + MOTOR_ADDRESS, 1, position);
			if (result == 1)
			{
				// double angle = (position / sensorResolution * TAU);
				// angle = _filterAngle(angle);
				// angle += angleOffset;
				// if (angle > M_PI)
				// 	angle -= TAU;
				// if (angle < -M_PI)
				// 	angle += TAU;
				// angle *= readRatio;
				// return angle;
				return position;
			}
			else
			{
				RCLCPP_ERROR(rclcpp::get_logger(CLASS_NAME),
							 "I2C Read Error during joint position read.");
				// throw std::runtime_error("I2C Read Error during joint position read. Exiting for safety.");
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
		int8_t data[2];

		if (type_ == TYPE_MOTOR)
		{
			const int LOW = 70;
			const int HIGH = 100;
			const int THRESHOLD = 30;

			if (effort > 100.0)
				effort = 100.0;
			else if (effort < -100.0)
				effort = -100.0;

			int8_t speed = effort;

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
                double magnitude = effort * 100.0;
                uint8_t value = floor(min_ + ((max_ - min_) * (magnitude / 100.0)));

                data[0] = id_ - 1;
                data[1] = value;

				uint8_t result = i2c_->writeData(id_ + BASE_SLAVE_ADDRESS, data);
				RCLCPP_INFO(rclcpp::get_logger(CLASS_NAME),
				            "write: %i; effort: %f; bytes: %i, %i",
							result, effort, data[0], data[1]);
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
