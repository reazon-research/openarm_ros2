#include <array>
#include <string>
#include <urdf/model.h>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "openarm_gravcomp_controller/gravcomp_controller.hpp"
#include "controller_interface/helpers.hpp"

namespace gravcomp_controller
{
GravCompController::GravCompController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn GravCompController::on_init()
{
  RCLCPP_INFO(get_node()->get_logger(), "GravCompController on_init");
  // not sure this should be here
  joint_positions_ = KDL::JntArray(7);
  gravity_torques_ = KDL::JntArray(7);
  std::string description_path = ament_index_cpp::get_package_share_directory(
    "openarm_bimanual_description"
  );
  std::string chain_root_link = "pedestal_link";
  std::string left_leaf_link = "left_link8";
  auto urdf_model = urdf::Model();
  if (!urdf_model.initFile(description_path + "/urdf/openarm_bimanual.urdf")) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse urdf file");
    return controller_interface::CallbackReturn::ERROR;
  }
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse kdl tree");
    return controller_interface::CallbackReturn::ERROR;
  }
  KDL::Chain kdl_chain;
  if (!kdl_tree.getChain(chain_root_link, left_leaf_link, kdl_chain)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get kdl chain");
    return controller_interface::CallbackReturn::ERROR;
  }
  chain_dynamics_ = std::make_unique<KDL::ChainDynParam>(kdl_chain, KDL::Vector(0, 0.0, -9.81));
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravCompController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_node()->get_logger(), "GravCompController on_activate");
  std::vector<std::string> joint_names = {"rev1", "rev2", "rev3", "rev4", "rev5", "rev6", "rev7"};

  // setup command interfaces
  std::string interface = "effort";
  if (!controller_interface::get_ordered_interfaces(
      command_interfaces_, joint_names, interface, joint_command_interface_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get command interfaces");
    return controller_interface::CallbackReturn::ERROR;
  }

  // setup state interfaces
  interface = hardware_interface::HW_IF_POSITION;
  if (!controller_interface::get_ordered_interfaces(
      state_interfaces_, joint_names, interface, joint_state_interface_))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get state interfaces");
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravCompController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_node()->get_logger(), "GravCompController on_deactivate");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravCompController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_node()->get_logger(), "GravCompController on_configure");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration GravCompController::command_interface_configuration()
const
{
  // effort controller
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  const std::string prefix = "";
  conf.names.push_back(prefix + "rev1/command");
  conf.names.push_back(prefix + "rev2/command");
  conf.names.push_back(prefix + "rev3/command");
  conf.names.push_back(prefix + "rev4/command");
  conf.names.push_back(prefix + "rev5/command");
  conf.names.push_back(prefix + "rev6/command");
  conf.names.push_back(prefix + "rev7/command");
  return conf;
}

controller_interface::InterfaceConfiguration GravCompController::state_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // FIXME: consider getting these from param
  const std::string prefix = "";
  conf.names.push_back(prefix + "rev1/position");
  conf.names.push_back(prefix + "rev2/position");
  conf.names.push_back(prefix + "rev3/position");
  conf.names.push_back(prefix + "rev4/position");
  conf.names.push_back(prefix + "rev5/position");
  conf.names.push_back(prefix + "rev6/position");
  conf.names.push_back(prefix + "rev7/position");
  return conf;
}

controller_interface::return_type GravCompController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  for (size_t i = 0; i < 7; ++i) {
    joint_positions_(i) = joint_state_interface_[i].get().get_value();
  }
  chain_dynamics_->JntToGravity(joint_positions_, gravity_torques_);
  double gravity_torque_scale = 1.2;

  for (size_t i = 0; i < 7; ++i) {
    joint_command_interface_[i].get().set_value(gravity_torques_(i) * gravity_torque_scale);
  }
}

} // namespace gravcomp_controller
