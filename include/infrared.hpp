/** Copyright 2024 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#ifndef TURBOPI__INFRARED_H
#define TURBOPI__INFRARED_H

#include <array>
#include <inttypes.h>

#include "i2c.hpp"

const uint8_t INFRARED_ADDRESS = 0x78;

// classname used in logging output
extern char const* const CLASS_NAME;

namespace turbopi
{
    /**
     * @brief Class to interface with the Infrared hardware
     */
	class Infrared
	{
		public:

            /**
             * @brief Construct a new Infrared object, empty/unused
             */
            Infrared();

            /**
             * @brief Construct a new Infrared object, handles initialization
             *
             * @param i2c_dev          I2C device minor number range 0-89
             * @param i2c_address      I2C slave address
             */
			Infrared(uint8_t i2c_dev, uint8_t i2c_address);

            /**
             * @brief Destroy the Infrared object, closes device handle
             */
			virtual ~Infrared();

            /**
             * @brief Get values from infrared sensors
             *
             * @return uint8_t values read from infrared sensors
             */
            std::array<uint8_t, 4> getValues();

		private:
			I2C *i2c_;
            uint8_t i2c_address_;

	};
}

#endif
