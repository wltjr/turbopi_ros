
#include <cstdio>
#include <cstring>
#include <iostream>
#include <unistd.h>

#include "sonar.hpp"

/**
 * @brief Test program to test out Sonar sensor, different colors, read value
 *
 * @return int program return status/value
 */
int main()
{
    auto sonar = turbopi::Sonar(1, 0x77);
    std::cout << "Distance " << sonar.getDistance() << std::endl;
    sonar.setRGBMode(0);
    sonar.setPixelColor(0,16711680); // #FF0000 - red
    sonar.setPixelColor(1,16711680);
    sleep(1);
    sonar.setPixelColor(0,255); // #0000FF - blue
    sonar.setPixelColor(1,255);
    sleep(1);
    sonar.setPixelColor(0,65280); // #00FF00 - green
    sonar.setPixelColor(1,65280);
    sleep(1);
    sonar.setPixelColor(0,0); // #000000 - black/off
    sonar.setPixelColor(1,0);
}
