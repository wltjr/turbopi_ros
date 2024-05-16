/** Copyright 2024 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#include <sys/ioctl.h>
#include <errno.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <syslog.h>

#include "rclcpp/rclcpp.hpp"

#include "i2c.hpp"

namespace turbopi
{
	I2C::I2C()
	{
    }

	I2C::I2C(uint8_t minor, int8_t address)
	{
		minor_ = minor;
		address_ = address;
		snprintf(busfile, sizeof(busfile), "/dev/i2c-%d", minor_);
		openfd();
	}

	I2C::~I2C()
	{
		close(fd);
	}

	uint8_t I2C::readBytes(int8_t register_number,
                           uint8_t buffer_size,
                           int8_t &value)
	{
		if (fd != -1)
		{
			int8_t buffer[buffer_size];

			uint8_t write_buffer_size = 1;
			int8_t write_buffer[write_buffer_size] = {0};
			write_buffer[0] = register_number;

			if (ioctl(fd, I2C_SLAVE, address_) < 0)
			{
				RCLCPP_ERROR(rclcpp::get_logger(CLASS_NAME),
							 "I2C ioctl error slave 0x%x", address_);
				return -1;
			}

			if (write(fd, write_buffer, write_buffer_size) != write_buffer_size)
			{
				RCLCPP_ERROR(rclcpp::get_logger(CLASS_NAME),
							 "[readBytes():write] I2C slave 0x%x failed to write to register 0x%x\nError: %d - %s",
							 address_, register_number, errno, strerror(errno));
				return (-1);
			}
			if (read(fd, buffer, buffer_size) != buffer_size)
			{
				RCLCPP_ERROR(rclcpp::get_logger(CLASS_NAME),
							 "[readBytes():read] Could not read from I2C slave 0x%x, register 0x%x\nError: %d - %s",
							 address_, register_number, errno, strerror(errno));
				return (-1);
			}
			else
			{
				value = 0;
				for (int i = 0; i < buffer_size; i++)
				{
					value =  value + buffer[i] ;
					RCLCPP_DEBUG(rclcpp::get_logger(CLASS_NAME),
								"i2c Buffer #%i: %i", i, buffer[i]);
				}
			}
		}
		else
		{
			RCLCPP_ERROR(rclcpp::get_logger(CLASS_NAME), "Device File not available. Aborting read");
			return (-1);
		}
		return (1);
	}

	uint8_t I2C::writeData(int8_t register_number, uint8_t data[2])
	{
		if (fd != -1)
		{
			uint8_t buff[2];
			buff[0] = register_number + data[0];
			buff[1] = data[1];

			int result = write(fd, buff, sizeof(buff));
			if (result != sizeof(buff))
			{
				RCLCPP_ERROR(rclcpp::get_logger(CLASS_NAME),
							 "[writeData():write] Failed to write to I2C Slave 0x%x @ register 0x%x\nError: %d - %s",
							 address_, register_number + data[0], errno, strerror(errno));
				return (-1);
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
			return (-1);
		}
		return 0;
	}

	void I2C::openfd()
	{
		if ((fd = open(busfile, O_RDWR)) < 0)
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
