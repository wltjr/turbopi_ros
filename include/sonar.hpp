/** Copyright 2024 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#ifndef TURBOPI__SONAR_H
#define TURBOPI__SONAR_H

#include <array>
#include <inttypes.h>
#include <vector>

#include "i2c.hpp"

const uint8_t SONAR_ADDRESS = 0x77;

// classname used in logging output
extern char const* const CLASS_NAME;

namespace turbopi
{
    /**
     * @brief Class to interface with the Sonar hardware
     */
	class Sonar
	{
		public:

            /**
             * @brief Construct a new Sonar object, empty/unused
             */
            Sonar();

            /**
             * @brief Construct a new Sonar object, handles initialization
             *
             * @param i2c_dev          I2C device minor number range 0-89
             * @param i2c_address      I2C slave address
             */
			Sonar(uint8_t i2c_dev, uint8_t i2c_address);

            /**
             * @brief Destroy the Sonar object, closes device handle
             */
			virtual ~Sonar();

            /**
             * @brief Get distance reading from sensor
             *
             * @return int distance reading from sensor
             */
            int getDistance();

            /**
             * @brief Get pixel color from sensor
             *
             * @param index          device index
             *
             * @return uint8_t pixel color from sensor
             */
            std::array<uint8_t, 3> getPixelColor(uint8_t index);

            /**
             * @brief Set sensor pixel color
             *
             * @param index          device index
             * @param rgb            rgb color
             */
            uint8_t setPixelColor(uint8_t index, uint32_t rgb);

            /**
             * @brief Set sensor pixel color
             *
             * @param mode          RGB mode range 0-2
             */
            void setRGBMode(uint8_t mode);

		private:
			I2C *i2c_;
            uint8_t i2c_address_;
            std::array<uint32_t, 2> pixels = {0};
            uint8_t rgb_mode_ = 2;

	};
}

#endif
