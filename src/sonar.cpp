/** Copyright 2024 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#include "sonar.hpp"

const char* CLASS_NAME = "Sonar";

namespace turbopi
{
    Sonar::Sonar()
    {
    }
 
    Sonar::Sonar(uint8_t i2c_dev, uint8_t i2c_address)
    {
        i2c_address_ = i2c_address;
        static I2C i2c = I2C(i2c_dev, i2c_address_);
        i2c_ = &i2c;
    }

    Sonar::~Sonar()
    {
    }

    int Sonar::getDistance()
    {
        uint8_t distance = 255;

        uint8_t result = i2c_->readBytes(i2c_address_, 2, distance);
        if (result != 1)
		{
            distance = 0;
        }

        return distance;
    }

    std::array<uint8_t, 3> Sonar::getPixelColor(uint8_t index)
    {
        if (index != 0 && index != 1)
            return {0,0,0};
        return {(uint8_t)((pixels[index] >> 16) & 0xFF),
                (uint8_t)((pixels[index] >> 8) & 0xFF),
                (uint8_t)((pixels[index]) & 0xFF)};
    }

    uint8_t Sonar::setPixelColor(uint8_t index, uint32_t rgb)
    {
        if (index != 0 && index != 1)
            return 0;

        uint8_t start_reg = (index == 0) ? 3:6;
		uint8_t data[2];

        data[0] = 0;
        data[1] = (uint8_t)(0xFF & (rgb >> 16));
        uint8_t result = i2c_->writeData(start_reg, data);
        data[0] = 0;
        data[1] = (uint8_t)(0xFF & (rgb >> 8));
        result = i2c_->writeData(start_reg+1, data);
        data[0] = 0;
        data[1] = (uint8_t)(0xFF & rgb);
        result = i2c_->writeData(start_reg+2, data);
        pixels[index] = rgb;

        return result;
    }

    void Sonar::setRGBMode(uint8_t mode)
    {
		uint8_t data[2];

        data[0] = 0;
        data[1] = mode;
        uint8_t result = i2c_->writeData(2, data);
        rgb_mode_ = mode;
    }

}
