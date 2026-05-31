// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TURBOPI__TURBOPI_HARDWARE_INTERFACE_HPP_
#define TURBOPI__TURBOPI_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"

#include "turbopi.hpp"

namespace turbopi_hardware_interface
{
    class TurboPiSystemHardware : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(TurboPiSystemHardware)

        /**
         * @brief Destroy the TurboPiSystemHardware object, call on_deactivate
         */
        ~TurboPiSystemHardware();

        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareComponentInterfaceParams &params) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        // params
        double hw_start_sec_;
        double hw_stop_sec_;

        // Servo trim offsets applied at the hardware layer (not visible to controller).
        // Loaded from ros2_control.xacro <param name="camera_pan_offset"> and
        // <param name="camera_tilt_offset">. Adjust to correct mechanical zero.
        double camera_pan_offset_  = 0.0;
        double camera_tilt_offset_ = 0.0;

        turbopi::TurboPi turbopi_;
        std::vector<double> hw_commands_;
        std::vector<double> hw_positions_;
        std::vector<double> hw_velocities_;

        // Battery voltage publisher (mV) – shared with battery_node via /battery_voltage_mv topic
        rclcpp::Node::SharedPtr battery_node_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr battery_pub_;
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr buzzer_sub_;
        int battery_pub_counter_ = 0;
    };

}

#endif
