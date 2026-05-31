/** Copyright 2025 William L Thomson Jr <w@wltjr.com>
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *
 *  C++ port of ros_robot_controller_sdk.py (Hiwonder Pi5 board SDK).
 *  Speaks the 0xAA 0x55 serial protocol to the STM32 co-processor.
 */

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

#include "board.hpp"

char const* const CLASS_NAME = "Board";

namespace turbopi
{

// ── CRC-8 table (identical to Python SDK) ─────────────────────────────────────

static const uint8_t crc8_table[256] = {
    0,   94,  188, 226, 97,  63,  221, 131, 194, 156, 126, 32,  163, 253, 31,  65,
    157, 195, 33,  127, 252, 162, 64,  30,  95,  1,   227, 189, 62,  96,  130, 220,
    35,  125, 159, 193, 66,  28,  254, 160, 225, 191, 93,  3,   128, 222, 60,  98,
    190, 224, 2,   92,  223, 129, 99,  61,  124, 34,  192, 158, 29,  67,  161, 255,
    70,  24,  250, 164, 39,  121, 155, 197, 132, 218, 56,  102, 229, 187, 89,  7,
    219, 133, 103, 57,  186, 228, 6,   88,  25,  71,  165, 251, 120, 38,  196, 154,
    101, 59,  217, 135, 4,   90,  184, 230, 167, 249, 27,  69,  198, 152, 122, 36,
    248, 166, 68,  26,  153, 199, 37,  123, 58,  100, 134, 216, 91,  5,   231, 185,
    140, 210, 48,  110, 237, 179, 81,  15,  78,  16,  242, 172, 47,  113, 147, 205,
    17,  79,  173, 243, 112, 46,  204, 146, 211, 141, 111, 49,  178, 236, 14,  80,
    175, 241, 19,  77,  206, 144, 114, 44,  109, 51,  209, 143, 12,  82,  176, 238,
    50,  108, 142, 208, 83,  13,  239, 177, 240, 174, 76,  18,  145, 207, 45,  115,
    202, 148, 118, 40,  171, 245, 23,  73,  8,   86,  180, 234, 105, 55,  213, 139,
    87,  9,   235, 181, 54,  104, 138, 212, 149, 203, 41,  119, 244, 170, 72,  22,
    233, 183, 85,  11,  136, 214, 52,  106, 43,  117, 151, 201, 74,  20,  246, 168,
    116, 42,  200, 150, 21,  75,  169, 247, 182, 232, 10,  84,  215, 137, 107, 53
};

// ── helpers ────────────────────────────────────────────────────────────────────

static void pack_le16(std::vector<uint8_t> &buf, uint16_t v)
{
    buf.push_back(static_cast<uint8_t>(v & 0xFF));
    buf.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
}

// ── Board ──────────────────────────────────────────────────────────────────────

Board::Board(const std::string &device, uint32_t baudrate)
{
    fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger(CLASS_NAME),
                     "Failed to open serial port %s: %s", device.c_str(), strerror(errno));
        return;
    }

    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger(CLASS_NAME),
                     "tcgetattr error: %s", strerror(errno));
        return;
    }

    // Raw mode
    cfmakeraw(&tty);

    // Map requested baud rate
    speed_t speed = B115200;
    switch (baudrate)
    {
        case 1000000: speed = B1000000; break;
        case 921600:  speed = B921600;  break;
        case 460800:  speed = B460800;  break;
        case 230400:  speed = B230400;  break;
        case 115200:  speed = B115200;  break;
        default:
            RCLCPP_WARN(rclcpp::get_logger(CLASS_NAME),
                        "Unsupported baud rate %u, defaulting to 115200", baudrate);
    }
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CRTSCTS;

    // Non-blocking read
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1; // 100 ms timeout

    if (tcsetattr(fd_, TCSANOW, &tty) != 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger(CLASS_NAME),
                     "tcsetattr error: %s", strerror(errno));
        return;
    }

    // Start background receive thread
    recv_thread_ = std::thread(&Board::recvTask, this);

    RCLCPP_INFO(rclcpp::get_logger(CLASS_NAME),
                "Board opened %s @ %u baud", device.c_str(), baudrate);
}

Board::~Board()
{
    enable_recv_.store(false);
    if (recv_thread_.joinable())
        recv_thread_.join();
    if (fd_ >= 0)
        close(fd_);
}

// ── CRC-8 ─────────────────────────────────────────────────────────────────────

uint8_t Board::crc8(const uint8_t *data, size_t len)
{
    uint8_t check = 0;
    for (size_t i = 0; i < len; i++)
        check = crc8_table[check ^ data[i]];
    return check;
}

// ── Low-level framing ─────────────────────────────────────────────────────────

void Board::bufWrite(PacketFunction func, const std::vector<uint8_t> &data)
{
    if (fd_ < 0)
        return;

    // Frame (mirrors Python SDK buf_write):
    //   0xAA 0x55 <func> <len> [data…] <crc8(func+len+data)>
    // The Python SDK buf_write does:
    //   buf = [0xAA, 0x55, int(func)]
    //   buf.append(len(data))
    //   buf.extend(data)
    //   buf.append(checksum_crc8(bytes(buf[2:])))   ← CRC of func+len+data
    std::vector<uint8_t> frame;
    frame.reserve(4 + data.size() + 1);
    frame.push_back(0xAA);
    frame.push_back(0x55);
    frame.push_back(static_cast<uint8_t>(func));   // byte 2
    frame.push_back(static_cast<uint8_t>(data.size())); // byte 3
    frame.insert(frame.end(), data.begin(), data.end());

    // CRC covers func, len, and data (bytes 2 onward)
    uint8_t chk = crc8(frame.data() + 2, frame.size() - 2);
    frame.push_back(chk);

    ssize_t written = write(fd_, frame.data(), frame.size());
    if (written < 0 || static_cast<size_t>(written) != frame.size())
    {
        RCLCPP_ERROR(rclcpp::get_logger(CLASS_NAME),
                     "Serial write error: %s", strerror(errno));
    }
}

// ── Motor control ─────────────────────────────────────────────────────────────

void Board::setMotorDuty(const std::vector<std::pair<uint8_t, float>> &speeds)
{
    // Sub-command 0x05 = set duty
    // Payload: 0x05, count, [id-1, float32_le …]
    std::vector<uint8_t> data;
    data.push_back(0x05);
    data.push_back(static_cast<uint8_t>(speeds.size()));
    for (auto &[id, duty] : speeds)
    {
        data.push_back(static_cast<uint8_t>(id - 1)); // 0-based
        // Pack float as 4 bytes little-endian
        float f = duty;
        uint8_t fb[4];
        std::memcpy(fb, &f, 4);
        data.insert(data.end(), fb, fb + 4);
    }
    bufWrite(PacketFunction::MOTOR, data);
}

void Board::setMotorSpeed(const std::vector<std::pair<uint8_t, float>> &speeds)
{
    // Sub-command 0x01 = set speed
    std::vector<uint8_t> data;
    data.push_back(0x01);
    data.push_back(static_cast<uint8_t>(speeds.size()));
    for (auto &[id, speed] : speeds)
    {
        data.push_back(static_cast<uint8_t>(id - 1)); // 0-based
        float f = speed;
        uint8_t fb[4];
        std::memcpy(fb, &f, 4);
        data.insert(data.end(), fb, fb + 4);
    }
    bufWrite(PacketFunction::MOTOR, data);
}

// ── Battery ───────────────────────────────────────────────────────────────────

void Board::enableReception(bool enable)
{
    enable_recv_.store(enable);
}

int Board::getBattery()
{
    return battery_mv_.load();
}

// ── RGB ───────────────────────────────────────────────────────────────────────

void Board::setRGB(const std::vector<std::array<uint8_t, 4>> &pixels)
{
    // Payload: 0x01, count, [idx-1, r, g, b …]
    std::vector<uint8_t> data;
    data.push_back(0x01);
    data.push_back(static_cast<uint8_t>(pixels.size()));
    for (auto &p : pixels)
    {
        data.push_back(static_cast<uint8_t>(p[0] - 1)); // 0-based index
        data.push_back(p[1]); // r
        data.push_back(p[2]); // g
        data.push_back(p[3]); // b
    }
    bufWrite(PacketFunction::RGB, data);
}

// ── Buzzer ────────────────────────────────────────────────────────────────────

void Board::setBuzzer(uint16_t freq, float on_time, float off_time, uint16_t repeat)
{
    uint16_t on_ms  = static_cast<uint16_t>(on_time  * 1000);
    uint16_t off_ms = static_cast<uint16_t>(off_time * 1000);
    std::vector<uint8_t> data;
    pack_le16(data, freq);
    pack_le16(data, on_ms);
    pack_le16(data, off_ms);
    pack_le16(data, repeat);
    bufWrite(PacketFunction::BUZZER, data);
}

// ── PWM servos ────────────────────────────────────────────────────────────────

void Board::pwmServoSetPosition(float duration,
                                const std::vector<std::pair<uint8_t, uint16_t>> &positions)
{
    uint16_t dur_ms = static_cast<uint16_t>(duration * 1000);
    std::vector<uint8_t> data;
    data.push_back(0x01);
    data.push_back(static_cast<uint8_t>(dur_ms & 0xFF));
    data.push_back(static_cast<uint8_t>((dur_ms >> 8) & 0xFF));
    data.push_back(static_cast<uint8_t>(positions.size()));
    for (auto &[id, pulse] : positions)
    {
        data.push_back(id);
        pack_le16(data, pulse);
    }
    bufWrite(PacketFunction::PWM_SERVO, data);
}

// ── Receive task ──────────────────────────────────────────────────────────────

void Board::recvTask()
{
    // Parser state machine – mirrors canonical Python SDK recv_task() in
    // turbopi_ros2_pi5/src/driver/sdk/sdk/ros_robot_controller_sdk.py EXACTLY.
    //
    // Wire format from STM32:
    //   0xAA 0x55 <FUNCTION> <LENGTH> <data[0..LENGTH-1]> <crc8>
    //
    // State order: STARTBYTE1 → STARTBYTE2 → FUNCTION → LENGTH → DATA → CHECKSUM
    //   (after STARTBYTE2 → FUNCTION state; after FUNCTION → LENGTH state)
    //
    // Python frame[] = [func, len, data...]
    //   frame[0] = func byte  (set when FUNCTION state fires)
    //   frame[1] = len byte   (set when LENGTH state fires)
    //   frame[2..] = data bytes (LENGTH bytes total)
    //
    // CRC = crc8(frame) covers [func, len, data...]
    //
    // Battery SYS packet (func=0x00, sub-cmd=0x04):
    //   Wire: 0xAA 0x55 <func=0x00> <len=3> <0x04> <mv_lo> <mv_hi> <crc>
    //   frame = [0x00, 3, 0x04, mv_lo, mv_hi]
    //   data = frame[2:] = [0x04, mv_lo, mv_hi]
    //   data[0] == 0x04 → battery sub-command

    enum class State : uint8_t
    {
        START1, START2, FUNCTION, LENGTH, DATA, CHECKSUM
    };

    State      state      = State::START1;
    uint8_t    func       = 0;
    uint8_t    length     = 0;
    uint8_t    recv_count = 0;
    std::vector<uint8_t> frame;  // [func, len, data...]

    while (enable_recv_.load() || fd_ >= 0)
    {
        if (!enable_recv_.load())
        {
            usleep(10000); // 10 ms
            continue;
        }

        uint8_t byte = 0;
        ssize_t n = read(fd_, &byte, 1);
        if (n <= 0)
            continue;

        switch (state)
        {
            case State::START1:
                if (byte == 0xAA)
                    state = State::START2;
                break;

            case State::START2:
                state = (byte == 0x55) ? State::FUNCTION : State::START1;
                break;

            case State::FUNCTION:
                if (byte < static_cast<uint8_t>(PacketFunction::NONE))
                {
                    func = byte;
                    frame.clear();
                    frame.push_back(func); // frame[0] = func
                    frame.push_back(0);    // frame[1] = placeholder for length
                    state = State::LENGTH;
                }
                else
                {
                    state = State::START1;
                }
                break;

            case State::LENGTH:
                length = byte;
                frame[1] = length;
                recv_count = 0;
                state = (length == 0) ? State::CHECKSUM : State::DATA;
                break;

            case State::DATA:
                frame.push_back(byte);
                if (++recv_count >= length)
                    state = State::CHECKSUM;
                break;

            case State::CHECKSUM:
            {
                // CRC covers entire frame[] = [func, len, data...]
                uint8_t expected = crc8(frame.data(), frame.size());
                if (expected == byte)
                {
                    PacketFunction pf = static_cast<PacketFunction>(func);
                    // frame[2:] = payload data bytes
                    // Matches Python: data = bytes(self.frame[2:])
                    const uint8_t *payload = frame.data() + 2;
                    size_t payload_len = frame.size() - 2;

                    if (pf == PacketFunction::SYS && payload_len >= 3 && payload[0] == 0x04)
                    {
                        // Battery: SYS sub-cmd 0x04, followed by uint16_t LE millivolts
                        uint16_t mv = static_cast<uint16_t>(payload[1]) |
                                      (static_cast<uint16_t>(payload[2]) << 8);
                        battery_mv_.store(static_cast<int>(mv));
                        RCLCPP_DEBUG(rclcpp::get_logger(CLASS_NAME),
                                     "Battery: %u mV", static_cast<unsigned>(mv));
                    }
                    else
                    {
                        RCLCPP_DEBUG(rclcpp::get_logger(CLASS_NAME),
                                     "RX func=0x%02X len=%u payload[0]=0x%02X payload_len=%zu",
                                     func, length,
                                     payload_len > 0 ? payload[0] : 0xFFu,
                                     payload_len);
                    }
                }
                else
                {
                    RCLCPP_WARN(rclcpp::get_logger(CLASS_NAME),
                                "CRC mismatch: got 0x%02X expected 0x%02X (func=0x%02X len=%u)",
                                byte, expected, func, length);
                }
                state = State::START1;
                break;
            }
        }
    }
}

} // namespace turbopi
