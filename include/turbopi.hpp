/** Copyright 2024 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *
 *  Updated for Pi5 / ROS Robot Controller (STM32) board.
 */

#ifndef TURBOPI__TURBOPI_H
#define TURBOPI__TURBOPI_H

#include <sstream>

#include "board.hpp"
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
             *        sensors, etc.  Opens /dev/rrc (Pi5 STM32 board).
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
             * @brief Set a Joint object by name
             *
             * @param joint a Joint object
             */
			void setJoint(turbopi::Joint const & joint);

            /**
             * @brief Get the latest battery voltage from the STM32.
             *
             * @return Battery voltage in mV, or -1 if not yet available.
             */
            int getBattery();

            /**
             * @brief Sound the buzzer via the STM32 board.
             *
             * @param freq      Frequency in Hz (e.g. 1900 = high, 400 = low)
             * @param on_time   Beep on duration in seconds
             * @param off_time  Beep off (gap) duration in seconds
             * @param repeat    Number of beep repetitions
             */
            void setBuzzer(uint16_t freq, float on_time, float off_time, uint16_t repeat);

	};
}

#endif
