
#include <iostream>

#include "battery.hpp"

/**
 * @brief Test program to test out Battery, read voltage value via Pi5 Board.
 *        Opens /dev/rrc (STM32 co-processor) and reads the battery voltage
 *        broadcasted over the serial UART protocol.
 *
 * @return int program return status/value
 */
int main()
{
    static turbopi::Board board;
    board.enableReception(true);

    auto battery = turbopi::Battery(board);
    std::cout << "Voltage " << battery.getVoltage() << std::endl;
}
