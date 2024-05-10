/** Copyright 2024 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#ifndef TURBOPI__I2C_H
#define TURBOPI__I2C_H

#include <inttypes.h>

// classname used in logging output
extern const char *CLASS_NAME;

namespace turbopi
{
    /**
     * @brief Class to interface with devices via I2C hardware interface
     */
	class I2C
	{
		public:

            /**
             * @brief Construct a new I2C object, handles initialization
             *
             * @param minor   I2C device minor number range 0-89
             * @param address I2C slave address
             */
			I2C(uint8_t minor, int8_t address);

            /**
             * @brief Destroy the I2C object, closes device handle
             */
			virtual ~I2C();

            /**
             * @brief Read bytes/value from a I2C component via register number
             *
             * @param register_number   I2C register number
             * @param buffer_size       read bytes buffer size
             * @param value             the value of bytes read
             * 
             * @return uint8_t          -1 for error or 0 for success
             */
			uint8_t readBytes(int8_t register_number,
                              uint8_t buffer_size,
                              int8_t &value);

            /**
             * @brief Write data to I2C component via register number
             *
             * @param register_number   I2C register number
             * @param data              the data to write
             * 
             * @return uint8_t          -1 for error or 0 for success
             */
			uint8_t writeData(int8_t register_number, int8_t data[2]);

		private:
			char busfile[64];
			int fd;
			int8_t minor_;
			uint8_t address_;

            /**
             * @brief Open file descriptor to the I2C device
             */
			void openfd();
	};
}

#endif
