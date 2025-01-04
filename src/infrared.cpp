/** Copyright 2024 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#include <iostream>

#include "infrared.hpp"

char const* const CLASS_NAME = "Infrared";

namespace turbopi
{
    Infrared::Infrared() = default;
 
    Infrared::Infrared(uint8_t i2c_dev, uint8_t i2c_address) :
        i2c_address_(i2c_address)
    {
        static auto i2c = I2C(i2c_dev, i2c_address_);
        i2c_ = &i2c;
    }

    Infrared::~Infrared() = default;

    std::array<uint8_t, 4> Infrared::getValues()
    {
        std::vector<uint8_t> data(1);
        std::array<uint8_t, 4> values = {0,0,0,0};

        if (i2c_->readBytes(0x01, 1, data))
        {
            switch (data[0])
            {
                case 0x07: values = {(uint8_t)1,0,0,0}; break; // sensor 1 - 0111
                case 0x0B: values = {0,(uint8_t)1,0,0}; break; // sensor 2 - 1011
                case 0x0D: values = {0,0,(uint8_t)1,0}; break; // sensor 3 - 1101
                case 0x0E: values = {0,0,0,(uint8_t)1}; break; // sensor 4 - 1110

                case 0x03: values = {(uint8_t)1,(uint8_t)1,0,0}; break; // sensor 1-2 - 0011
                case 0x09: values = {0,(uint8_t)1,(uint8_t)1,0}; break; // sensor 2-3 - 1001
                case 0x0C: values = {0,0,(uint8_t)1,(uint8_t)1}; break; // sensor 3-4 - 1100
                case 0x01: values = {(uint8_t)1,(uint8_t)1,(uint8_t)1,0}; break; // sensor 1-3 - 0001
                case 0x08: values = {0,(uint8_t)1,(uint8_t)1,(uint8_t)1}; break; // sensor 2-4 - 1000
                case 0x00: values = {(uint8_t)1,(uint8_t)1,(uint8_t)1,(uint8_t)1}; break; // sensor 1-4 - 0000
                default: break;
            }
        }

        return values;
    }

}
