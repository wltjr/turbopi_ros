/** Copyright 2025 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *
 *  Pi5 / ROS Robot Controller board (STM32) communication layer.
 *  Replaces the Pi4 I2C-based HiwonderSDK/Board.py with a C++ port of
 *  ros_robot_controller_sdk.py that speaks the 0xAA 0x55 serial protocol.
 */

#ifndef TURBOPI__BOARD_H
#define TURBOPI__BOARD_H

#include <array>
#include <atomic>
#include <cstdint>
#include <optional>
#include <string>
#include <thread>
#include <vector>

// classname used in logging output
extern char const* const CLASS_NAME;

namespace turbopi
{
    /**
     * @brief Packet function IDs – mirror of PacketFunction in the Python SDK.
     */
    enum class PacketFunction : uint8_t
    {
        SYS       = 0,
        LED       = 1,
        BUZZER    = 2,
        MOTOR     = 3,
        PWM_SERVO = 4,
        BUS_SERVO = 5,
        KEY       = 6,
        IMU       = 7,
        GAMEPAD   = 8,
        SBUS      = 9,
        OLED      = 10,
        RGB       = 11,
        NONE      = 12
    };

    /**
     * @brief C++ port of the Python Board class from ros_robot_controller_sdk.py.
     *        Communicates with the STM32 co-processor on the Pi5 board via UART.
     */
    class Board
    {
    public:
        /**
         * @brief Construct a new Board object and open the serial port.
         *
         * @param device   Serial device path (default /dev/rrc)
         * @param baudrate UART baud rate  (default 1 000 000)
         */
        explicit Board(const std::string &device   = "/dev/rrc",
                       uint32_t           baudrate  = 1000000);

        /**
         * @brief Destroy the Board object, stops receive thread and closes port.
         */
        ~Board();

        // Non-copyable, non-movable (owns a thread and atomic state)
        Board(const Board &)            = delete;
        Board &operator=(const Board &) = delete;
        Board(Board &&)                 = delete;
        Board &operator=(Board &&)      = delete;

        // ── Motor control ─────────────────────────────────────────────────────

        /**
         * @brief Set motor duty-cycle values.
         *
         * @param speeds  Vector of (1-based motor id, duty [-100..100]) pairs.
         */
        void setMotorDuty(const std::vector<std::pair<uint8_t, float>> &speeds);

        /**
         * @brief Set motor speed values (closed-loop, if supported by firmware).
         *
         * @param speeds  Vector of (1-based motor id, speed m/s) pairs.
         */
        void setMotorSpeed(const std::vector<std::pair<uint8_t, float>> &speeds);

        // ── Battery ───────────────────────────────────────────────────────────

        /**
         * @brief Enable / disable the receive thread.
         *
         * @param enable true = start receiving, false = pause
         */
        void enableReception(bool enable = true);

        /**
         * @brief Get the latest battery voltage reported by the STM32.
         *
         * @return Battery voltage in mV, or -1 if not yet available.
         */
        int getBattery();

        // ── RGB ───────────────────────────────────────────────────────────────

        /**
         * @brief Set RGB LED colours.
         *
         * @param pixels  Vector of (1-based index, r, g, b) tuples.
         */
        void setRGB(const std::vector<std::array<uint8_t, 4>> &pixels);

        // ── Buzzer ────────────────────────────────────────────────────────────

        /**
         * @brief Sound the buzzer.
         *
         * @param freq      Frequency in Hz
         * @param on_time   On-time  in seconds
         * @param off_time  Off-time in seconds
         * @param repeat    Repeat count
         */
        void setBuzzer(uint16_t freq,
                       float    on_time,
                       float    off_time,
                       uint16_t repeat = 1);

        // ── PWM servos ────────────────────────────────────────────────────────

        /**
         * @brief Move PWM servos to target positions.
         *
         * @param duration   Motion duration in seconds
         * @param positions  Vector of (1-based servo id, pulse width μs) pairs.
         */
        void pwmServoSetPosition(float duration,
                                 const std::vector<std::pair<uint8_t, uint16_t>> &positions);

    private:
        int         fd_      = -1;   ///< serial file descriptor
        std::atomic<bool> enable_recv_{false};
        std::atomic<int>  battery_mv_{-1};
        std::thread recv_thread_;

        // ── Internal helpers ──────────────────────────────────────────────────

        /**
         * @brief Compute CRC-8 over a byte range (same table as Python SDK).
         */
        static uint8_t crc8(const uint8_t *data, size_t len);

        /**
         * @brief Build and write a framed packet to the serial port.
         *
         * Frame format: 0xAA 0x55 <func> <len> [data…] <crc8>
         */
        void bufWrite(PacketFunction func, const std::vector<uint8_t> &data);

        /**
         * @brief Background thread that reads incoming packets and updates
         *        internal state (battery, IMU, …).
         */
        void recvTask();
    };

} // namespace turbopi

#endif // TURBOPI__BOARD_H
