################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.20)
project(turbopi_ros)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 23)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-g -Wall -Wextra -Wpedantic)
endif()


################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
set(INCLUDE_DEPENDS
    ament_cmake
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    controller_manager
    joint_limits
    geometry_msgs
    sensor_msgs
    std_msgs
    trajectory_msgs
)

foreach(Dependency IN ITEMS ${INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach()


################################################################################
# Build
################################################################################
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

# ── Shared object libraries ────────────────────────────────────────────────────
#
# Pi5 change: the I2C transport layer for motors / battery / servos is replaced
# by the Board UART serial library.  Sonar and Infrared keep their own I2C
# access (via smbus / Linux I2C-dev) because the sonar sensor (0x77) and the
# four-infrared sensor (0x78) are identical hardware on both Pi4 and Pi5.

# Board: UART serial driver for the Pi5 STM32 co-processor
add_library(board SHARED src/board.cpp)

# I2C: low-level Linux I2C-dev driver still used by sonar and infrared sensors
# (the sonar sensor 0x77 and four-infrared sensor 0x78 are I2C on both Pi4 and Pi5)
add_library(i2c SHARED src/i2c.cpp)

# Hardware sensors (still I2C on Pi5)
add_library(infrared SHARED src/infrared.cpp)
add_library(sonar    SHARED src/sonar.cpp)

# Higher-level modules that now depend on Board instead of I2C
add_library(battery SHARED src/battery.cpp)
add_library(teleop_turbopi SHARED src/teleop_turbopi.cpp)
add_library(turbopi_hardware_interface
    SHARED
    src/joint.cpp
    src/turbopi.cpp
    src/turbopi_hardware_interface.cpp
)

# ── Executable binaries ────────────────────────────────────────────────────────
add_executable(battery_monitor_node src/battery_monitor_node.cpp)
add_executable(battery_node         src/battery_node.cpp)
add_executable(infrared_node        src/infrared_node.cpp)
add_executable(sonar_node           src/sonar_node.cpp)
add_executable(teleop_turbopi_node  src/teleop_turbopi_node.cpp)
add_executable(test_battery         src/tests/test_battery.cpp)
add_executable(test_infrared        src/tests/test_infrared.cpp)
add_executable(test_sonar           src/tests/test_sonar.cpp)

# ── ROS 2 dependencies ─────────────────────────────────────────────────────────
ament_target_dependencies(battery_monitor_node      ${INCLUDE_DEPENDS})
ament_target_dependencies(board                     ${INCLUDE_DEPENDS})
ament_target_dependencies(i2c                       ${INCLUDE_DEPENDS})
ament_target_dependencies(teleop_turbopi            ${INCLUDE_DEPENDS})
ament_target_dependencies(turbopi_hardware_interface ${INCLUDE_DEPENDS})

# ── Link executable binaries to shared object libraries ────────────────────────

# Sonar / infrared: still on I2C (unchanged from Pi4)
target_link_libraries(infrared i2c)
target_link_libraries(infrared_node infrared)
target_link_libraries(sonar i2c)
target_link_libraries(sonar_node sonar)

# Battery library (still used by test_battery)
target_link_libraries(battery board)
target_link_libraries(test_battery battery board)
# battery_monitor_node is a pure ROS subscriber of /battery topic – no hardware links needed

# battery_node is now a pure ROS subscriber – no direct hardware access.
# It subscribes to /battery_voltage_mv published by the hardware interface.
ament_target_dependencies(battery_node ${INCLUDE_DEPENDS})
target_link_libraries(test_infrared i2c infrared)
target_link_libraries(test_sonar i2c sonar)

target_link_libraries(teleop_turbopi_node teleop_turbopi)
target_link_libraries(turbopi_hardware_interface board)


################################################################################
# Plugin
################################################################################
pluginlib_export_plugin_description_file(hardware_interface turbopi_hardware_interface.xml)


################################################################################
# Install
################################################################################
install(
    DIRECTORY config description launch2 meshes worlds
    DESTINATION share/${PROJECT_NAME}
)

set(INSTALL_TARGETS
    board
    i2c
    battery
    battery_monitor_node
    battery_node
    infrared
    infrared_node
    sonar
    sonar_node
    teleop_turbopi
    teleop_turbopi_node
    turbopi_hardware_interface
)

foreach(target IN ITEMS ${INSTALL_TARGETS})
    install(TARGETS ${target}
        EXPORT ${target}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
    )
endforeach()


################################################################################
# Macro for ament package
################################################################################
ament_package()
