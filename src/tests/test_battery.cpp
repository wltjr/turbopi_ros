
#include <iostream>

#include "battery.hpp"

/**
 * @brief Test program to test out Battery, read voltage value
 *
 * @return int program return status/value
 */
int main()
{
    auto battery = turbopi::Battery(1, BATTERY_ADDRESS);
    std::cout << "Voltage " << battery.getVoltage() << std::endl;
}
