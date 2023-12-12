#ifndef TURBOPI__I2C_H
#define TURBOPI__I2C_H

#include <inttypes.h>

#define BUFFER_SIZE 1

extern const char *CLASS_NAME;

namespace turbopi
{
	class I2C
	{
		public:
			uint8_t dataBuffer[BUFFER_SIZE];

			I2C(int, int);
			virtual ~I2C();
			uint8_t readBytes(uint8_t registerNumber, uint8_t bufferSize, uint16_t &position);
			uint8_t writeData(uint8_t registerNumber, int8_t data[2]);

		private:
			int _i2caddr;
			int _i2cbus;
			char busfile[64];
			int fd;

			void openfd();
	};
}

#endif
