/** Copyright 2024 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#include <stdexcept>

#include "rclcpp/rclcpp.hpp"

#include "turbopi.hpp"
#include "joint.hpp"

namespace turbopi
{
	TurboPi::TurboPi()
	{
        static auto i2c_ = I2C(1, BASE_SLAVE_ADDRESS);

		//base
		base.joints[0] = Joint(1, TYPE_MOTOR, i2c_);
		base.joints[0].name = "front_left_wheel_joint";
		base.joints[1] = Joint(2, TYPE_MOTOR, i2c_);
		base.joints[1].name = "front_right_wheel_joint";
		base.joints[2] = Joint(3, TYPE_MOTOR, i2c_);
		base.joints[2].name = "rear_left_wheel_joint";
		base.joints[3] = Joint(4, TYPE_MOTOR, i2c_);
		base.joints[3].name = "rear_right_wheel_joint";

		//camera
		camera.joints[0] = Joint(5, TYPE_SERVO, i2c_);
		camera.joints[0].name = "camera_joint";
		camera.joints[0].sensorResolution = 128;
		camera.joints[0].setLimits(0, 115);
		camera.joints[1] = Joint(6, TYPE_SERVO, i2c_);
		camera.joints[1].name = "camera_frame_joint";
		camera.joints[1].sensorResolution = 128;
		camera.joints[1].setLimits(0, 180);
	}

	TurboPi::~TurboPi() = default;

	Joint TurboPi::getJoint(std::string const & jointName)
	{
		int numJointsBase = base.joints.size();
		for (int i = 0; i < numJointsBase; i++)
		{
			if (base.joints[i].name == jointName)
			{
				return base.joints[i];
			}
		}

		int numJointsCamera = camera.joints.size();
		for (int i = 0; i < numJointsCamera; i++)
		{
			if (camera.joints[i].name == jointName)
			{
				return camera.joints[i];
			}
		}

		std::cout << "Could not find joint with name " << jointName << std::endl;

		return Joint();
	}

	void TurboPi::setJoint(Joint const & joint)
	{
		bool foundJoint = false;

		int numJointsBase = base.joints.size();
		for (int i = 0; i < numJointsBase; i++)
		{
			if (base.joints[i].name == joint.name)
			{
				foundJoint = true;
				base.joints[i] = joint;
			}
		}

		int numJointsCamera = camera.joints.size();
		for (int i = 0; i < numJointsCamera; i++)
		{
			if (camera.joints[i].name == joint.name)
			{
				foundJoint = true;
				camera.joints[i] = joint;
			}
		}

		if (foundJoint == false)
		    std::cout << "Could not find joint with name " << joint.name << std::endl;
	}

}
