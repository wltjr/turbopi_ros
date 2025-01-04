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
                case 0x39: values = {(uint8_t)1,0,0,0}; break; // sensor 1
                case 0x38: values = {0,(uint8_t)1,0,0}; break; // sensor 2
                case 0x36: values = {0,0,(uint8_t)1,0}; break; // sensor 3
                case 0x32: values = {0,0,0,(uint8_t)1}; break; // sensor 4

                case 0x37: values = {(uint8_t)1,(uint8_t)1,0,0}; break; // sensor 1-2
                case 0x34: values = {0,(uint8_t)1,(uint8_t)1,0}; break; // sensor 2-3
                case 0x33: values = {(uint8_t)1,(uint8_t)1,(uint8_t)1,0}; break; // sensor 1-3
                case 0x2e: values = {0,0,(uint8_t)1,(uint8_t)1}; break; // sensor 3-4
                case 0x2c: values = {0,(uint8_t)1,(uint8_t)1,(uint8_t)1}; break; // sensor 2-4
                case 0x2b: values = {(uint8_t)1,(uint8_t)1,(uint8_t)1,(uint8_t)1}; break; // sensor 1-4
                default: break;
            }
        }

        return values;
    }

}
