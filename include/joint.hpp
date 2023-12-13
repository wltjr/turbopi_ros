#ifndef TURBOPI__JOINT_H
#define TURBOPI__JOINT_H

#include <sstream>

#define BASE_SLAVE_ADDRESS 0x7A
#define CAMERA_ADDRESS 21
#define MOTOR_ADDRESS 31

#define ACTUATOR_TYPE_NONE -1
#define ACTUATOR_TYPE_MOTOR 0
#define ACTUATOR_TYPE_SERVO 1

extern const char* CLASS_NAME;

namespace turbopi
{
	class Joint
	{
		private:
			uint8_t _motorId = 0;
			uint8_t _actuatorType = 0;
			int8_t _getSlaveAddress();
			uint8_t _minServoValue = 0;
			uint8_t _maxServoValue = 75;
			double _previousEffort;
			double _filterAngle(double angle);
			int _angleReads = 0;
			static const int _filterPrevious = 3;
			double _previousAngles[_filterPrevious];

			void _prepareI2CWrite(int8_t result[2], double effort);
			void _prepareI2CRead(int8_t result[2]);
		public:
			std::string name;
			Joint();
			Joint(uint8_t motorId);
			~Joint();
			double sensorResolution = 1024;
			double angleOffset = 0;
			double readRatio = 1;
			uint8_t getMotorId();
			void setMotorId(uint8_t motorId);
			void setActuatorType(uint8_t actuatorType);
			void setServoLimits(uint8_t minValue, uint8_t maxValue);
			int getActuatorType();
			double getPreviousEffort();
			void actuate(double effort, uint8_t duration);
			double readAngle();
	};
}

#endif