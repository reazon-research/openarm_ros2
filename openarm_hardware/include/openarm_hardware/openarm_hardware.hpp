// Copyright (c) 2025, Reazon Holdings, Inc.
// Copyright (c) 2025, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#ifndef OPENARM_HARDWARE__OPENARM_HARDWARE_HPP_
#define OPENARM_HARDWARE__OPENARM_HARDWARE_HPP_

#include <string>
#include <vector>
#include <memory>

#include "openarm_hardware/visibility_control.h"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "canbus.hpp"
#include "motor.hpp"
#include "motor_control.hpp"

namespace openarm_hardware
{

std::vector<DM_Motor_Type> motor_types{DM_Motor_Type::DM4340, DM_Motor_Type::DM4340, DM_Motor_Type::DM4340, DM_Motor_Type::DM4340, DM_Motor_Type::DM4310, DM_Motor_Type::DM4310, DM_Motor_Type::DM4310};
std::vector<uint16_t> can_device_ids{0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
std::vector<uint16_t> can_master_ids{0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};
static const Control_Type CONTROL_MODE = Control_Type::MIT;
static const std::size_t ARM_DOF = 7;
static const std::size_t GRIPPER_DOF = 1;
static const std::size_t TOTAL_DOF = ARM_DOF + GRIPPER_DOF;
static const std::array<double, TOTAL_DOF> KP = {120.0, 80.0, 20.0, 55.0, 5.0, 5.0, 5.0, 0.5};
static const std::array<double, TOTAL_DOF> KD = {1.25, 0.17, 0.015, 0.07, 0.07, 0.05, 0.05, 0.01};
static const std::array<double, TOTAL_DOF> SLOW_KP = {10.0, 10.0, 10.0, 7.5, 5.0, 5.0, 5.0, 0.5};
static const double START_POS_TOLERANCE_RAD = 0.1;
static const double POS_JUMP_TOLERANCE_RAD = 3.1415 / 2.0;

static const bool USING_GRIPPER = true;
static const double GRIPPER_REFERENCE_GEAR_RADIUS_M = 0.00853;
static const int GRIPPER_INDEX = TOTAL_DOF - 1;

class OpenArmHW : public hardware_interface::SystemInterface
{
public:
  OpenArmHW();
  
  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  std::size_t curr_dof = ARM_DOF; // minus gripper
private:
  std::string prefix_;
  std::unique_ptr<CANBus> canbus_;
  std::unique_ptr<MotorControl> motor_control_;
  std::vector<double> pos_commands_;
  std::vector<double> pos_states_;
  std::vector<double> vel_commands_;
  std::vector<double> vel_states_;
  std::vector<double> tau_ff_commands_;
  std::vector<double> tau_states_;
  std::vector<std::unique_ptr<Motor>> motors_;

  void refresh_motors();
  bool disable_torque_;
};

}  // namespace openarm_hardware

#endif  // OPENARM_HARDWARE__OPENARM_HARDWARE_HPP_
