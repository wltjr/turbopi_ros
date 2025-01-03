/** Copyright 2024 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#ifndef TURBOPI__I2C_H
#define TURBOPI__I2C_H

#include <inttypes.h>
#include <vector>

// classname used in logging output
extern char const* const CLASS_NAME;

namespace turbopi
{
    /**
     * @brief Class to interface with devices via I2C hardware interface
     */
	class I2C
	{
		public:

            /**
             * @brief Construct a new I2C object, empty/unused
             */
            I2C();

            /**
             * @brief Construct a new I2C object, handles initialization
             *
             * @param minor   I2C device minor number range 0-89
             * @param address I2C slave address
             */
			I2C(uint8_t minor, uint8_t address);

            /**
             * @brief Destroy the I2C object, closes device handle
             */
			virtual ~I2C();

            /**
             * @brief Read bytes/value from a I2C component via register number
             *
             * @param register_number   I2C register number
             * @param buffer_size       read bytes buffer size
             * @param buffer            a vector of bytes read
             * 
             * @return uint8_t          -1 for error or 0 for success
             */
			uint8_t readBytes(uint8_t register_number,
                              uint8_t buffer_size,
                              std::vector<uint8_t> &buffer);

            /**
             * @brief Write data to I2C component via register number
             *
             * @param register_number   I2C register number
             * @param data              the data to write
             * 
             * @return uint8_t          -1 for error or 0 for success
             */
			uint8_t writeData(uint8_t register_number,  std::array<uint8_t, 2> data);

		private:
			int fd;
			std::string busfile;
			uint8_t minor_;
			uint8_t address_;

            /**
             * @brief Open file descriptor to the I2C device
             */
			void openfd();
	};
}

#endif
