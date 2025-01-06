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

# shared object libraries, lib*.so
add_library(i2c SHARED src/i2c.cpp)
add_library(battery SHARED src/battery.cpp)
add_library(infrared SHARED src/infrared.cpp)
add_library(sonar SHARED src/sonar.cpp)
add_library(teleop_turbopi SHARED src/teleop_turbopi.cpp)
add_library(turbopi_hardware_interface
    SHARED
    src/joint.cpp
    src/turbopi.cpp
    src/turbopi_hardware_interface.cpp
)

# executable binaries
add_executable(battery_node src/battery_node.cpp)
add_executable(infrared_node src/infrared_node.cpp)
add_executable(sonar_node src/sonar_node.cpp)
add_executable(teleop_turbopi_node src/teleop_turbopi_node.cpp)
add_executable(test_battery src/tests/test_battery.cpp)
add_executable(test_infrared src/tests/test_infrared.cpp)
add_executable(test_sonar src/tests/test_sonar.cpp)

# add ROS dependencies
ament_target_dependencies(i2c ${INCLUDE_DEPENDS})
ament_target_dependencies(teleop_turbopi ${INCLUDE_DEPENDS})
ament_target_dependencies(turbopi_hardware_interface ${INCLUDE_DEPENDS})

# link executable binaries to shared object libraries
target_link_libraries(battery i2c)
target_link_libraries(battery_node i2c battery)
target_link_libraries(infrared i2c)
target_link_libraries(infrared_node i2c infrared)
target_link_libraries(sonar i2c)
target_link_libraries(sonar_node sonar)
target_link_libraries(teleop_turbopi_node teleop_turbopi)
target_link_libraries(test_battery i2c battery)
target_link_libraries(test_infrared i2c infrared)
target_link_libraries(test_sonar i2c sonar)
target_link_libraries(turbopi_hardware_interface i2c)


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
    i2c
    battery
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
