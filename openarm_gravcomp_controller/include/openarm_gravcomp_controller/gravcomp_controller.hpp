#pragma once
#include "openarm_gravcomp_controller/visibility_control.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <memory>
#include <kdl/chaindynparam.hpp>

namespace gravcomp_controller
{

class GravCompController : public controller_interface::ControllerInterface
{
public:
  OPENARM_GRAVCOMP_CONTROLLER_PUBLIC
  GravCompController();

  OPENARM_GRAVCOMP_CONTROLLER_PUBLIC
  ~GravCompController() = default;

  // let manager? know the comand interface that this controller will use
  OPENARM_GRAVCOMP_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  // let manager? know the state interface that this controller will use
  OPENARM_GRAVCOMP_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  OPENARM_GRAVCOMP_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  OPENARM_GRAVCOMP_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  OPENARM_GRAVCOMP_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  OPENARM_GRAVCOMP_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  OPENARM_GRAVCOMP_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  template<typename T>
  using InterfaceReferences = std::vector<std::reference_wrapper<T>>;

  InterfaceReferences<hardware_interface::LoanedCommandInterface> joint_command_interface_;
  InterfaceReferences<hardware_interface::LoanedStateInterface> joint_state_interface_;
  KDL::JntArray joint_positions_;
  KDL::JntArray gravity_torques_;

  std::unique_ptr<KDL::ChainDynParam> chain_dynamics_;
};

}  // namespace gravcomp_controller
