/** Copyright 2024 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#include <sys/ioctl.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>

#include "rclcpp/rclcpp.hpp"

#include "i2c.hpp"

namespace turbopi
{
	I2C::I2C() = default;

	I2C::I2C(uint8_t minor, uint8_t address) :
		minor_(minor), address_(address)
	{
		busfile = std::format("/dev/i2c-{}", minor_);
		openfd();
	}

	I2C::~I2C()
	{
		close(fd);
	}

	uint8_t I2C::readBytes(uint8_t register_number,
                           uint8_t buffer_size,
                           std::vector<uint8_t> &buffer)
	{
		if (fd != -1)
		{
			uint8_t write_buffer_size = 1;
			std::vector<uint8_t>  write_buffer(write_buffer_size);
			write_buffer[0] = register_number;
			uint8_t* buf = write_buffer.data();

			if (ioctl(fd, I2C_SLAVE, address_) < 0)
			{
				RCLCPP_ERROR(rclcpp::get_logger(CLASS_NAME),
							 "I2C ioctl error slave 0x%x", address_);
				return -1;
			}

			if (write(fd, buf, write_buffer_size) != write_buffer_size)
			{
				RCLCPP_ERROR(rclcpp::get_logger(CLASS_NAME),
							 "[readBytes():write] I2C slave 0x%x failed to write to register 0x%x\nError: %d - %s",
							 address_, register_number, errno, strerror(errno));
				return -1;
			}

            buf = buffer.data();
			if (read(fd, buf, buffer_size) != buffer_size)
			{
				RCLCPP_ERROR(rclcpp::get_logger(CLASS_NAME),
							 "[readBytes():read] Could not read from I2C slave 0x%x, register 0x%x\nError: %d - %s",
							 address_, register_number, errno, strerror(errno));
				return -1;
			}
			else
			{
				for (int i = 0; i < buffer_size; i++)
				{
					RCLCPP_DEBUG(rclcpp::get_logger(CLASS_NAME),
								"i2c Buffer #%i: %i", i, buffer[i]);
				}
			}
		}
		else
		{
			RCLCPP_ERROR(rclcpp::get_logger(CLASS_NAME), "Device File not available. Aborting read");
			return -1;
		}
		return 1;
	}

	uint8_t I2C::writeData(uint8_t register_number, std::array<uint8_t, 2> data)
	{
		if (fd != -1)
		{
			data[0] = register_number + data[0];
			uint8_t* buf = data.data();
			ssize_t size = data.size();

			if (write(fd, buf, size) != size)
			{
				RCLCPP_ERROR(rclcpp::get_logger(CLASS_NAME),
							 "[writeData():write] Failed to write to I2C Slave 0x%x @ register 0x%x\nError: %d - %s",
							 address_, register_number + data[0], errno, strerror(errno));
				return -1;
			}
			else
			{
				RCLCPP_DEBUG(rclcpp::get_logger(CLASS_NAME),
							 "Wrote to I2C Slave 0x%x @ register 0x%x", address_, register_number);
			}
		}
		else
		{
			RCLCPP_ERROR(rclcpp::get_logger(CLASS_NAME), "Device File not available. Aborting write");
			return -1;
		}
		return 1;
	}

	void I2C::openfd()
	{
		if ((fd = open(busfile.c_str(), O_RDWR)) < 0)
		{
			RCLCPP_ERROR(rclcpp::get_logger(CLASS_NAME),
						 "Couldn't open I2C Bus %d [openfd():open %s]", minor_, strerror(errno));
		}
		if (ioctl(fd, I2C_SLAVE, address_) < 0)
		{
			RCLCPP_ERROR(rclcpp::get_logger(CLASS_NAME),
						 "I2C slave %d failed [openfd():ioctl %s]", address_, strerror(errno));
		}
	}
}
