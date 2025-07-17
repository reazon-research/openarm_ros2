// Copyright 2025 Reazon Holdings, Inc.
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

#include "openarm_hardware/v10_simple_hardware.hpp"

#include <chrono>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

namespace openarm_hardware {

OpenArmV10HW::OpenArmV10HW() = default;

bool OpenArmV10HW::parse_config(const hardware_interface::HardwareInfo& info) {
  // Parse CAN interface (default: can0)
  auto it = info.hardware_parameters.find("can_interface");
  can_interface_ = (it != info.hardware_parameters.end()) ? it->second : "can0";

  // Parse gripper enable (default: true for V10)
  it = info.hardware_parameters.find("enable_gripper");
  enable_gripper_ = (it == info.hardware_parameters.end())
                        ? ENABLE_GRIPPER
                        : (it->second == "true");

  RCLCPP_INFO(rclcpp::get_logger("OpenArmV10HW"),
              "Configuration: CAN=%s, gripper=%s", can_interface_.c_str(),
              enable_gripper_ ? "enabled" : "disabled");

  return true;
}

hardware_interface::CallbackReturn OpenArmV10HW::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  std::cout << "on_init" << std::endl;

  std::cout << "on_init" << std::endl;

  // Parse configuration
  if (!parse_config(info)) {
    return CallbackReturn::ERROR;
  }

  std::cout << "on_init" << std::endl;

  // Validate joint count (7 arm joints + optional gripper)
  size_t expected_joints = ARM_DOF + (enable_gripper_ ? 1 : 0);
  if (info_.joints.size() != expected_joints) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArmV10HW"),
                 "Expected %zu joints, got %zu", expected_joints,
                 info_.joints.size());
    return CallbackReturn::ERROR;
  }

  // Initialize OpenArm with CAN-FD enabled (like full_arm.cpp)
  RCLCPP_INFO(rclcpp::get_logger("OpenArmV10HW"),
              "Initializing OpenArm on %s...", can_interface_.c_str());
  openarm_ =
      std::make_unique<openarm::can::socket::OpenArm>(can_interface_, true);

  // Initialize arm motors with V10 defaults
  openarm_->init_arm_motors(DEFAULT_MOTOR_TYPES, DEFAULT_SEND_CAN_IDS,
                            DEFAULT_RECV_CAN_IDS);

  // Initialize gripper if enabled
  if (enable_gripper_) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArmV10HW"), "Initializing gripper...");
    openarm_->init_gripper_motor(openarm::damiao_motor::DMMotorType::DM4310,
                                 0x08, 0x18);
  }

  // Initialize state and command vectors
  const size_t total_joints = info_.joints.size();
  pos_commands_.resize(total_joints, 0.0);
  vel_commands_.resize(total_joints, 0.0);
  tau_commands_.resize(total_joints, 0.0);
  pos_states_.resize(total_joints, 0.0);
  vel_states_.resize(total_joints, 0.0);
  tau_states_.resize(total_joints, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("OpenArmV10HW"),
              "OpenArm V10 Simple HW initialized successfully");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArmV10HW::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Set callback mode to ignore during configuration
  openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::IGNORE);
  openarm_->refresh_all();
  openarm_->recv_all();

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OpenArmV10HW::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &pos_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &vel_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
        &tau_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
OpenArmV10HW::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &pos_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &vel_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
        &tau_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn OpenArmV10HW::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArmV10HW"), "Activating OpenArm V10...");

  // Set callback mode to state and enable all motors (like full_arm.cpp)
  openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
  openarm_->enable_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->recv_all();

  // Return to zero position smoothly
  return_to_zero();

  RCLCPP_INFO(rclcpp::get_logger("OpenArmV10HW"), "OpenArm V10 activated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArmV10HW::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArmV10HW"),
              "Deactivating OpenArm V10...");

  // Disable all motors (like full_arm.cpp exit)
  openarm_->disable_all();
  openarm_->recv_all();

  RCLCPP_INFO(rclcpp::get_logger("OpenArmV10HW"), "OpenArm V10 deactivated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type OpenArmV10HW::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Receive all motor states
  openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
  openarm_->recv_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::IGNORE);

  // Read arm joint states
  const auto& arm_motors = openarm_->get_arm().get_motors();
  for (size_t i = 0; i < ARM_DOF && i < arm_motors.size(); ++i) {
    pos_states_[i] = arm_motors[i].get_position();
    vel_states_[i] = arm_motors[i].get_velocity();
    tau_states_[i] = arm_motors[i].get_torque();
  }

  // Read gripper state if enabled
  if (enable_gripper_ && info_.joints.size() > ARM_DOF) {
    const auto& gripper_motors = openarm_->get_gripper().get_motors();
    if (!gripper_motors.empty()) {
      pos_states_[ARM_DOF] = gripper_motors[0].get_position();
      vel_states_[ARM_DOF] = gripper_motors[0].get_velocity();
      tau_states_[ARM_DOF] = gripper_motors[0].get_torque();
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArmV10HW::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Control arm motors with MIT control
  std::vector<openarm::damiao_motor::MITParam> arm_params;
  for (size_t i = 0; i < ARM_DOF; ++i) {
    arm_params.push_back({DEFAULT_KP[i], DEFAULT_KD[i], pos_commands_[i],
                          vel_commands_[i], tau_commands_[i]});
  }
  openarm_->get_arm().mit_control_all(arm_params);

  // Control gripper if enabled
  if (enable_gripper_ && info_.joints.size() > ARM_DOF) {
    openarm_->get_gripper().mit_control_all(
        {{5.0, 1.0, pos_commands_[ARM_DOF], vel_commands_[ARM_DOF],
          tau_commands_[ARM_DOF]}});
  }

  return hardware_interface::return_type::OK;
}

void OpenArmV10HW::return_to_zero() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArmV10HW"),
              "Returning to zero position...");

  // // Return arm to zero with MIT control
  // std::vector<openarm::damiao_motor::MITParam> arm_params;
  // for (size_t i = 0; i < ARM_DOF; ++i) {
  //   arm_params.push_back({DEFAULT_KP[i], DEFAULT_KD[i], 0.0, 0.0, 0.0});
  // }
  // openarm_->get_arm().mit_control_all(arm_params);

  // // Return gripper to zero if enabled
  // if (enable_gripper_) {
  //   openarm_->get_gripper().mit_control_all({{5.0, 1.0, 0.0, 0.0, 0.0}});
  // }

  // // Wait for motors to settle
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  openarm_->recv_all();
}

}  // namespace openarm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(openarm_hardware::OpenArmV10HW,
                       hardware_interface::SystemInterface)