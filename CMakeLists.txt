################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.8)
project(turbopi_ros)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-g -Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
set(INCLUDE_DEPENDS
  hardware_interface
  # control_msgs
  # std_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  # controller_interface
  controller_manager
  joint_limits
)

find_package(ament_cmake REQUIRED)
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

add_library(turbopi_hardware_interface
  SHARED
  src/i2c.cpp
  src/joint.cpp
  src/turbopi_hardware_interface.cpp
  src/turbopi.cpp
)
ament_target_dependencies(turbopi_hardware_interface ${INCLUDE_DEPENDS})

target_link_libraries(turbopi_hardware_interface)


################################################################################
# Plugin
################################################################################
pluginlib_export_plugin_description_file(hardware_interface turbopi_hardware_interface.xml)

################################################################################
# Install
################################################################################
install(
  DIRECTORY config description launch2 worlds
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS turbopi_hardware_interface
  EXPORT turbopi_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
################################################################################
# Macro for ament package
################################################################################
ament_package()
