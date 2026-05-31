/** Copyright 2025 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *
 *  Updated for Pi5 / ROS Robot Controller (STM32) board.
 *  Battery voltage is read from the STM32 via the Board serial class.
 *
 *  The STM32 firmware periodically broadcasts a SYS packet (sub-cmd 0x04)
 *  containing the battery voltage in millivolts (uint16_t, little-endian).
 *  Board::recvTask() stores the latest value in battery_mv_ (atomic<int>).
 *  Board::getBattery() returns that value (-1 if not yet received).
 *
 *  getVoltage() is intentionally non-blocking: it reads whatever battery_mv_
 *  holds at call-time and returns 0 V when no reading is available yet.
 *  The battery_node timer fires every 5 s, giving the STM32 plenty of time
 *  to broadcast the first packet before the first publish.
 */

#include "rclcpp/rclcpp.hpp"

#include "battery.hpp"

char const* const CLASS_NAME = "Battery";

namespace turbopi
{
    Battery::Battery() = default;

    Battery::Battery(Board &board) :
        board_(&board)
    {
    }

    Battery::~Battery() = default;

    float Battery::getVoltage()
    {
        if (board_ == nullptr)
            return 0.0f;

        int mv = board_->getBattery();

        if (mv < 0)
        {
            RCLCPP_WARN(rclcpp::get_logger(CLASS_NAME),
                        "Battery voltage not yet available from STM32");
            return 0.0f;
        }

        return static_cast<float>(mv) / 1000.0f;
    }

}
