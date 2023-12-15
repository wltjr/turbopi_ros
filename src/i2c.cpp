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
	I2C::I2C(uint8_t bus, int8_t address)
	{
		_i2cbus = bus;
		_i2caddr = address;
		snprintf(busfile, sizeof(busfile), "/dev/i2c-%d", bus);
		openfd();
	}

	I2C::~I2C()
	{
		close(fd);
	}

	uint8_t I2C::readBytes(int8_t registerNumber, uint8_t bufferSize, int8_t &position)
	{
		if (fd != -1)
		{
			int8_t buff[bufferSize];

			uint8_t writeBufferSize = 1;
			int8_t writeBuffer[writeBufferSize] = {0};
			writeBuffer[0] = registerNumber;

			if (ioctl(fd, I2C_SLAVE, _i2caddr) < 0)
			{
				RCLCPP_ERROR(rclcpp::get_logger(CLASS_NAME),
							 "I2C ioctl error slave 0x%x", _i2caddr);
				return -1;
			}

			if (write(fd, writeBuffer, writeBufferSize) != writeBufferSize)
			{
				RCLCPP_ERROR(rclcpp::get_logger(CLASS_NAME),
							 "[readBytes():write] I2C slave 0x%x failed to write to register 0x%x\nError: %d - %s",
							 _i2caddr, registerNumber, errno, strerror(errno));
				return (-1);
			}
			if (read(fd, buff, bufferSize) != bufferSize)
			{
				RCLCPP_ERROR(rclcpp::get_logger(CLASS_NAME),
							 "[readBytes():read] Could not read from I2C slave 0x%x, register 0x%x\nError: %d - %s",
							 _i2caddr, registerNumber, errno, strerror(errno));
				return (-1);
			}
			else
			{
				position = 0;
				for (int i = 0; i < bufferSize; i++)
				{
					position =  position + buff[i] ;
					RCLCPP_DEBUG(rclcpp::get_logger(CLASS_NAME),
								"i2c Buffer #%i: %i", i, buff[i]);
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

	uint8_t I2C::writeData(int8_t registerNumber, int8_t data[2])
	{
		if (fd != -1)
		{
			int8_t buff[2];
			buff[0] = registerNumber + data[0];
			buff[1] = data[1];

			int result = write(fd, buff, sizeof(buff));
			if (result != sizeof(buff))
			{
				RCLCPP_ERROR(rclcpp::get_logger(CLASS_NAME),
							 "[writeData():write] Failed to write to I2C Slave 0x%x @ register 0x%x\nError: %d - %s",
							 _i2caddr, registerNumber + data[0], errno, strerror(errno));
				return (-1);
			}
			else
			{
				RCLCPP_DEBUG(rclcpp::get_logger(CLASS_NAME),
							 "Wrote to I2C Slave 0x%x @ register 0x%x", _i2caddr, registerNumber);
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
						 "Couldn't open I2C Bus %d [openfd():open %s]", _i2cbus, strerror(errno));
		}
		if (ioctl(fd, I2C_SLAVE, _i2caddr) < 0)
		{
			RCLCPP_ERROR(rclcpp::get_logger(CLASS_NAME),
						 "I2C slave %d failed [openfd():ioctl %s]", _i2caddr, strerror(errno));
		}
	}
}
