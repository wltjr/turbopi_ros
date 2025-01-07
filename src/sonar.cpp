/** Copyright 2024 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#include "rclcpp/rclcpp.hpp"

#include "sonar.hpp"

char const* const CLASS_NAME = "Sonar";

namespace turbopi
{
    Sonar::Sonar() = default;
 
    Sonar::Sonar(uint8_t i2c_dev, uint8_t i2c_address) :
        i2c_address_(i2c_address)
    {
        static auto i2c = I2C(i2c_dev, i2c_address_);
        i2c_ = &i2c;
    }

    Sonar::~Sonar() = default;

    int Sonar::getDistance()
    {
        int distance = 0;
        std::vector<uint8_t> data(2);

        if (i2c_->readBytes(0, 2, data))
            distance = (data[1] << 8) | data[0];

        if (distance > 5000)
            distance = 5000;

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
        uint8_t result;
        uint8_t start_reg;
        std::array<uint8_t, 2> data;

        if (index != 0 && index != 1)
            return 0;

        start_reg = (index == 0) ? 3:6;

        data[0] = 0;
        data[1] = (uint8_t)(0xFF & (rgb >> 16));
        result = i2c_->writeData(start_reg, data);
        if (!result)
            RCLCPP_ERROR(rclcpp::get_logger(CLASS_NAME),
                        "set pixel color write error on blue: %i", data[1]);

        data[1] = (uint8_t)(0xFF & (rgb >> 8));
        result = i2c_->writeData(start_reg+1, data);
        if (!result)
            RCLCPP_ERROR(rclcpp::get_logger(CLASS_NAME),
                        "set pixel color write error on green: %i", data[1]);

        data[1] = (uint8_t)(0xFF & rgb);
        result = i2c_->writeData(start_reg+2, data);
        if (!result)
            RCLCPP_ERROR(rclcpp::get_logger(CLASS_NAME),
                        "set pixel color write error on red: %i", data[1]);

        pixels[index] = rgb;

        return result;
    }

    void Sonar::setRGBMode(uint8_t mode)
    {
        uint8_t result;
        std::array<uint8_t, 2> data;

        data[0] = 0;
        data[1] = mode;
        result = i2c_->writeData(2, data);
        if (result)
            rgb_mode_ = mode;
        else
            RCLCPP_ERROR(rclcpp::get_logger(CLASS_NAME),
                        "change mode write error: %i", data[1]);
    }

}
