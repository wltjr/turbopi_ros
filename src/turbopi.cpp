#include <stdexcept>

#include "rclcpp/rclcpp.hpp"

#include "turbopi.hpp"
#include "joint.hpp"

namespace turbopi
{
	TurboPi::TurboPi()
	{
		//base
		base.joints[0].name = "front_left_wheel_joint";
		base.joints[0].setActuatorType(ACTUATOR_TYPE_MOTOR);
		base.joints[0].setMotorId(1);
		base.joints[1].name = "front_right_wheel_joint";
		base.joints[1].setActuatorType(ACTUATOR_TYPE_MOTOR);
		base.joints[1].setMotorId(2);
		base.joints[2].name = "rear_left_wheel_joint";
		base.joints[2].setActuatorType(ACTUATOR_TYPE_MOTOR);
		base.joints[2].setMotorId(3);
		base.joints[3].name = "rear_right_wheel_joint";
		base.joints[3].setActuatorType(ACTUATOR_TYPE_MOTOR);
		base.joints[3].setMotorId(4);

		//camera
		camera.joints[0].name = "joint_camera_horizontal";
		camera.joints[0].setActuatorType(ACTUATOR_TYPE_SERVO);
		camera.joints[0].setMotorId(5);
		camera.joints[0].sensorResolution = 128;
		camera.joints[1].name = "joint_camera_vertical";
		camera.joints[1].setActuatorType(ACTUATOR_TYPE_SERVO);
		camera.joints[1].setMotorId(6);
		camera.joints[1].sensorResolution = 128;
	}

	TurboPi::~TurboPi()
	{

	}

	Joint TurboPi::getJoint(std::string jointName)
	{
		int numJointsBase = sizeof(base.joints) / sizeof(base.joints[0]);
		for (int i = 0; i < numJointsBase; i++)
		{
			if (base.joints[i].name == jointName)
			{
				return base.joints[i];
			}
		}

		int numJointsCamera = sizeof(camera.joints) / sizeof(camera.joints[0]);
		for (int i = 0; i < numJointsCamera; i++)
		{
			if (camera.joints[i].name == jointName)
			{
				return camera.joints[i];
			}
		}

		throw std::runtime_error("Could not find joint with name " + jointName);
	}

	void TurboPi::setJoint(Joint joint)
	{
		bool foundJoint = false;

		int numJointsBase = sizeof(base.joints) / sizeof(base.joints[0]);
		for (int i = 0; i < numJointsBase; i++)
		{
			if (base.joints[i].name == joint.name)
			{
				foundJoint = true;
				base.joints[i] = joint;
			}
		}

		int numJointsCamera = sizeof(camera.joints) / sizeof(camera.joints[0]);
		for (int i = 0; i < numJointsCamera; i++)
		{
			if (camera.joints[i].name == joint.name)
			{
				foundJoint = true;
				camera.joints[i] = joint;
			}
		}

		if (foundJoint == false)
		{
			throw std::runtime_error("Could not find joint with name " + joint.name);
		}
	}

}
