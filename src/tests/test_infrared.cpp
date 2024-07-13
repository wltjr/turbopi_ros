
#include <cstdio>
#include <cstring>
#include <iostream>
#include <unistd.h>

#include "infrared.hpp"

/**
 * @brief Test program to test out infrared sensors
 *
 * @return int program return status/value
 */
int main()
{
    auto infrared = turbopi::Infrared(1, INFRARED_ADDRESS);

    while (true)
    {
        auto values = infrared.getValues();
        std::cout << "Sensor 1 " << +values[0] 
                << ", Sensor 2 " << +values[1] 
                << ", Sensor 3 " << +values[2]  
                << ", Sensor 4 " << +values[3] << std::endl << std::endl;
                sleep(0.5);
    }
}
