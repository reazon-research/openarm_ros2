#include "openarm_mujoco_hardware/openarm_mujoco_hardware.hpp"

namespace mujoco_hardware_interface{
hardware_interface::CallbackReturn MujocoHardware::on_init(const hardware_interface::HardwareInfo& info) {
    KP_ = 100.0;
    KD_ = 10.0;
    address_ = boost::asio::ip::make_address("127.0.0.1");
    endpoint_ = boost::asio::ip::tcp::endpoint{address_, kWebsocketPort};

    // allocate space for joint states
    const int DOF = 16;
    
    qpos_.resize(DOF, 0.0);
    qvel_.resize(DOF, 0.0);
    qtau_.resize(DOF, 0.0);

    cmd_qpos_.resize(DOF, 0.0);
    cmd_qvel_.resize(DOF, 0.0);
    cmd_qtau_ff_.resize(DOF, 0.0);

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoHardware::on_configure(const rclcpp_lifecycle::State& previous_state) {
    boost::beast::error_code ec;

    acceptor_.open(endpoint_.protocol(), ec);
    if (ec) {
        throw std::runtime_error("Open error: " + ec.message());
    }

    acceptor_.set_option(net::socket_base::reuse_address(true), ec);
    if (ec) {
        throw std::runtime_error("Set_option error: " + ec.message());
    }

    acceptor_.bind(endpoint_, ec);
    if (ec) {
        throw std::runtime_error("Bind error: " + ec.message());
    }

    acceptor_.listen(boost::asio::socket_base::max_listen_connections, ec);
    if (ec) {
        throw std::runtime_error("Listen error: " + ec.message());
    }

    read(rclcpp::Time(0), rclcpp::Duration(0, 0));
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoHardware::on_cleanup(const rclcpp_lifecycle::State& previous_state) {
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoHardware::on_shutdown(const rclcpp_lifecycle::State& previous_state) {
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoHardware::on_activate(const rclcpp_lifecycle::State& previous_state) {
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoHardware::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoHardware::on_error(const rclcpp_lifecycle::State& previous_state) {
    return hardware_interface::CallbackReturn::SUCCESS;
}
std::vector<hardware_interface::StateInterface> MujocoHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for(size_t i = 0; i < qpos_.size(); ++i){
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.at(i).name, hardware_interface::HW_IF_POSITION, &qpos_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.at(i).name, hardware_interface::HW_IF_VELOCITY, &qvel_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.at(i).name, hardware_interface::HW_IF_EFFORT, &qtau_[i]));
    }
    return state_interfaces;
}
std::vector<hardware_interface::CommandInterface> MujocoHardware::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < qpos_.size(); ++i) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints.at(i).name, hardware_interface::HW_IF_POSITION, &cmd_qpos_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints.at(i).name, hardware_interface::HW_IF_VELOCITY, &cmd_qvel_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints.at(i).name, hardware_interface::HW_IF_EFFORT, &cmd_qtau_ff_[i]));
    }
    return command_interfaces;
}
hardware_interface::return_type MujocoHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period) {
    // Read state from the last recieived websocket message
    // Why decouple this? The simulation might be paused, and we want to read the last state
    return hardware_interface::return_type::OK;
}
hardware_interface::return_type MujocoHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period) {
    // send a websocket message
    for(size_t i = 0; i < cmd_qpos_.size(); ++i) {
        sim_MIT_control(i);
    }
    return hardware_interface::return_type::OK;
}

void MujocoHardware::sim_MIT_control(const int interface_index) {
    const double qpos_error = cmd_qpos_[interface_index] - qpos_[interface_index];
    const double qvel_error = cmd_qvel_[interface_index] - qvel_[interface_index];
    const double qtau_ff = cmd_qtau_ff_[interface_index];

    const double cmd_torque = KP_ * qpos_error + KD_ * qvel_error + qtau_ff;
    
    std::cout << "torque: "<< cmd_torque << std::endl;
    // TODO: Send cmd_torque to the simulation environment
}

};
