/** Copyright 2025 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *
 *  Updated for Pi5 / ROS Robot Controller (STM32) board.
 *  Battery voltage is now read from the STM32 via the Board serial class
 *  instead of direct I2C ADC register reads.
 */

#ifndef TURBOPI__BATTERY_H
#define TURBOPI__BATTERY_H

#include <inttypes.h>

#include "board.hpp"

// classname used in logging output
extern char const* const CLASS_NAME;

namespace turbopi
{
    /**
     * @brief Class to interface with the Battery hardware via the Pi5 Board
     *        (STM32 co-processor).  Battery voltage is reported by the STM32
     *        over the serial UART in millivolts.
     */
	class Battery
	{
		public:

            /**
             * @brief Construct a new Battery object, empty/unused
             */
            Battery();

            /**
             * @brief Construct a new Battery object.
             *
             * @param board  reference to the shared Board serial interface
             */
			Battery(Board &board);

            /**
             * @brief Destroy the Battery object
             */
			virtual ~Battery();

            /**
             * @brief Get voltage reading from battery (in Volts).
             *        Reads the millivolt value reported by the STM32.
             *
             * @return float voltage reading in Volts, or 0.0 if not yet available
             */
            float getVoltage();

		private:
            Board *board_ = nullptr;
	};
}

#endif
