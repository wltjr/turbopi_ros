/** Copyright 2024 William L Thomson Jr <w@wltjr.com>
 * 
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 */

#include <memory>

#include "teleop_turbopi.hpp"

/**
 * @brief Node executable wrapper for the shared object
 * 
 * @param argc the count of arguments passed to the program on start
 * @param argv the argument values passed to the program on start
 * @return int program return code
 */
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_unique<teleop_turbopi::TurboPi>(rclcpp::NodeOptions()));

    rclcpp::shutdown();

    return 0;
}
