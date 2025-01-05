/** Copyright 2025 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#ifndef TURBOPI__BATTERY_H
#define TURBOPI__BATTERY_H

#include <inttypes.h>
#include <vector>

#include "i2c.hpp"

const uint8_t BATTERY_ADDRESS = 0x7A;

// classname used in logging output
extern char const* const CLASS_NAME;

namespace turbopi
{
    /**
     * @brief Class to interface with the Battery hardware
     */
	class Battery
	{
		public:

            /**
             * @brief Construct a new Battery object, empty/unused
             */
            Battery();

            /**
             * @brief Construct a new Battery object, handles initialization
             *
             * @param i2c_dev          I2C device minor number range 0-89
             * @param i2c_address      I2C slave address
             */
			Battery(uint8_t i2c_dev, uint8_t i2c_address);

            /**
             * @brief Destroy the Battery object, closes device handle
             */
			virtual ~Battery();

            /**
             * @brief Get voltage reading from battery
             *
             * @return float voltage reading from battery
             */
            float getVoltage();

		private:
			I2C *i2c_;
            uint8_t i2c_address_;

	};
}

#endif
