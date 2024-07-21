/** Copyright 2024 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#ifndef TURBOPI__TURBOPI_H
#define TURBOPI__TURBOPI_H

#include <sstream>

#include "i2c.hpp"
#include "segment.hpp"

extern char const* const CLASS_NAME;

namespace turbopi
{
    /**
     * @brief Class that represents the total robot, all joints, sensors, etc.
     */
	class TurboPi
	{
		public:
			Segment<5> base;
			Segment<2> camera;

            /**
             * @brief Construct a new Robot object and initializes all joints,
             *        sensors, etc
             */
			TurboPi();

            /**
             * @brief Destroy the Robot object, empty/unused
             */
			~TurboPi();

            /**
             * @brief Get a Joint object by name
             *
             * @param name name of the joint
             * 
             * @return Joint
             */
			Joint getJoint(std::string const & name);

            /**
             * @brief Get a Joint object by name
             *
             * @param joint a Joint object
             */
			void setJoint(turbopi::Joint const & joint);

	};
}

#endif