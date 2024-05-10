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
#include "i2c.hpp"

namespace turbopi
{
	Joint::Joint()
	{
	}

	Joint::Joint(uint8_t id, uint8_t type)
	{
		this->id_ = id;
		this->type_ = type;
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

			I2C i2cSlave = I2C(1, _getSlaveAddress());
			uint8_t result = i2cSlave.readBytes(id_ - 1 + MOTOR_ADDRESS, 1, position);
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
		if (type_ == TYPE_MOTOR)
		{
			if (effort > 100.0)
				effort = 100.0;
			if (effort < -100.0)
				effort = -100.0;

			int8_t data[2];

			_prepareI2CWrite(data, effort);
			int8_t slaveAddress = _getSlaveAddress();
			I2C i2cSlave = I2C(1, slaveAddress);
			uint8_t result = i2cSlave.writeData(MOTOR_ADDRESS, data);
			RCLCPP_INFO(rclcpp::get_logger(CLASS_NAME),
			            "Result: [%i]; effort: [%f]; motor: %i, speed: %i",
						result, effort, data[0], data[1]);
		}
		else if (type_ == TYPE_SERVO)
		{
			if (effort != _previousEffort)
			{
				int8_t data[2];
				_prepareI2CWrite(data, effort);
				int8_t slaveAddress = _getSlaveAddress();
				I2C i2cSlave = I2C(1, slaveAddress);
				uint8_t result = i2cSlave.writeData(id_ + slaveAddress, data);
				RCLCPP_INFO(rclcpp::get_logger(CLASS_NAME),
				            "Result: [%i]; effort: [%f]; bytes: %i, %i",
							result, effort, data[0], data[1]);
			}
		}

		_previousEffort = effort;
	}

	int8_t Joint::_getSlaveAddress()
	{
		// wheels
		if (id_ > 0 && id_ <= 4)
		{
			return BASE_SLAVE_ADDRESS;
		}
		// camera horizontal/vertical
		else if (id_ > 4 && id_ <= 6)
		{
			return BASE_SLAVE_ADDRESS;
		}
		else
		{
			RCLCPP_ERROR(rclcpp::get_logger(CLASS_NAME), "Invalid MotorID: %i", id_);
			return -1;
		}
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

	void Joint::_prepareI2CWrite(int8_t addr_value[2], double effort)
	{
		if (type_ == TYPE_MOTOR)
		{
			const int LOW = 70;
			const int HIGH = 100;
			const int THRESHOLD = 30;

			if (effort > 100.0)
				effort = 100.0;
			if (effort < -100.0)
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
			if(id_ & 1)
				speed = -speed;

			addr_value[0] = id_ - 1;
			addr_value[1] = speed;
		}
		else if (type_ == TYPE_SERVO)
		{
			double magnitude = effort * 100.0;
			uint8_t value = floor(min_ + ((max_ - min_) * (magnitude / 100.0)));

			addr_value[0] = id_ - 1;
			addr_value[1] = value;
		}
	}

	int Joint::getType()
	{
		return type_;
	}
}
