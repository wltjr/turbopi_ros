/** Copyright 2025 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "battery.hpp"
char const* const CLASS_NAME = "Battery";

namespace turbopi
{
    Battery::Battery() = default;
 
    Battery::Battery(uint8_t i2c_dev, uint8_t i2c_address) :
        i2c_address_(i2c_address)
    {
        static auto i2c = I2C(i2c_dev, i2c_address_);
        i2c_ = &i2c;
    }

    Battery::~Battery() = default;

    float Battery::getVoltage()
    {
        float voltages = 0;
        std::time_t previous = 0;
        std::vector<uint8_t> data = {0, 255};

        // take avg of 3 readings
        for (int count = 0; count < 3;)
        {
            std::time_t now = std::chrono::system_clock::to_time_t(
                std::chrono::system_clock::now());

            if (std::difftime(now, previous) < 1)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }

            previous = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now()) + 1;

            while (data[1] == 255)
            {
                if (!i2c_->readBytes(0, 2, data))
                {
                    data[0] = 0;
                    data[1] = 0;
                }
            }

            voltages += (float)((data[1] << 8) | data[0]);
            count++;
        }

        // return avg of 3 / 1000
        return voltages/3000.0;
    }

}
