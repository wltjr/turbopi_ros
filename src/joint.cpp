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

	Joint::Joint(uint8_t motorId)
	{
		setMotorId(motorId);
	}

	Joint::~Joint()
	{
	}

	void Joint::setActuatorType(uint8_t actuatorType)
	{
		this->_actuatorType = actuatorType;
	}

	uint8_t Joint::getMotorId()
	{
		return this->_motorId;
	}

	void Joint::setMotorId(uint8_t motorId)
	{
		this->_motorId = motorId;
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
		if (_actuatorType == ACTUATOR_TYPE_MOTOR)
		{
			uint16_t position;
			const int TAU = M_PI + M_PI;

			I2C i2cSlave = I2C(1, _getSlaveAddress());
			uint8_t result = i2cSlave.readBytes(_motorId + MOTOR_ADDRESS, 4, position);
			if (result == 1)
			{
				double angle = (position / sensorResolution * TAU);
				angle = _filterAngle(angle);
				angle += angleOffset;
				if (angle > M_PI)
					angle -= TAU;
				if (angle < -M_PI)
					angle += TAU;
				angle *= readRatio;
				return angle;
			}
			else
			{
				RCLCPP_ERROR(rclcpp::get_logger(CLASS_NAME),
							 "I2C Read Error during joint position read.");
				// throw std::runtime_error("I2C Read Error during joint position read. Exiting for safety.");
				return 0;
			}
		}
		else if (_actuatorType == ACTUATOR_TYPE_SERVO)
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
		if (_actuatorType == ACTUATOR_TYPE_MOTOR)
		{
			if (effort > 100.0)
				effort = 100.0;
			if (effort < -100.0)
				effort = -100.0;

			int8_t data[2];

			_prepareI2CWrite(data, effort);
			uint8_t slaveAddress = _getSlaveAddress();
			I2C i2cSlave = I2C(1, slaveAddress);
			uint8_t result = i2cSlave.writeData(MOTOR_ADDRESS, data);
			RCLCPP_INFO(rclcpp::get_logger(CLASS_NAME),
			            "Result: [%i]; effort: [%f]; motor: %i, speed: %i",
						result, effort, data[0], data[1]);
		}
		else if (_actuatorType == ACTUATOR_TYPE_SERVO)
		{
			if (effort != _previousEffort)
			{
				int8_t data[2];
				_prepareI2CWrite(data, effort);
				uint8_t slaveAddress = _getSlaveAddress();
				I2C i2cSlave = I2C(1, slaveAddress);
				uint8_t result = i2cSlave.writeData(_motorId + slaveAddress, data);
				RCLCPP_INFO(rclcpp::get_logger(CLASS_NAME),
				            "Result: [%i]; effort: [%f]; bytes: %i, %i",
							result, effort, data[0], data[1]);
			}
		}

		_previousEffort = effort;
	}

	uint8_t Joint::_getSlaveAddress()
	{
		// wheels
		if (_motorId > 0 && _motorId <= 4)
		{
			return BASE_SLAVE_ADDRESS;
		}
		// camera horizontal/vertical
		else if (_motorId > 4 && _motorId <= 6)
		{
			return BASE_SLAVE_ADDRESS;
		}
		else
		{
			RCLCPP_ERROR(rclcpp::get_logger(CLASS_NAME), "Invalid MotorID: %i", _motorId);
			return -1;
		}
	}

	void Joint::setServoLimits(uint8_t minValue, uint8_t maxValue)
	{
		this->_minServoValue = minValue;
		this->_maxServoValue = maxValue;
	}

	double Joint::getPreviousEffort()
	{
		return this->_previousEffort;
	}

	void Joint::_prepareI2CWrite(int8_t result[2], double effort)
	{
		if (_actuatorType == ACTUATOR_TYPE_MOTOR)
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
			if(_motorId & 1)
				speed = -speed;

			result[0] = _motorId - 1;
			result[1] = speed;
		}
		else if (_actuatorType == ACTUATOR_TYPE_SERVO)
		{
			double magnitude = effort * 100.0;
			uint8_t servoValue = floor(_minServoValue + ((_maxServoValue - _minServoValue) * (magnitude / 100.0)));

			result[0] = _motorId - 1;
			result[1] = servoValue;
		}
	}

	int Joint::getActuatorType()
	{
		return _actuatorType;
	}
}
