#include "openarm_mujoco_hardware/openarm_mujoco_hardware.hpp"

namespace mujoco_hardware_interface{

hardware_interface::CallbackReturn MujocoHardware::on_init(const hardware_interface::HardwareInfo& /*info*/) {
    KP_ = 100.0;
    KD_ = 10.0;
    ioc_ = boost::asio::io_context(1);
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

hardware_interface::CallbackReturn MujocoHardware::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    boost::beast::error_code ec;

    acceptor_.open(endpoint_.protocol(), ec);
    if (ec) {
        throw std::runtime_error("open error: " + ec.message());
    }

    acceptor_.set_option(boost::asio::socket_base::reuse_address(true), ec);
    if (ec) {
        throw std::runtime_error("enable address reuse error: " + ec.message());
    }

    acceptor_.bind(endpoint_, ec);
    if (ec) {
        throw std::runtime_error("bind error: " + ec.message());
    }

    acceptor_.listen(boost::asio::socket_base::max_listen_connections, ec);
    if (ec) {
        throw std::runtime_error("listen error: " + ec.message());
    }

    acceptor_.async_accept(
        [this](boost::beast::error_code ec, boost::asio::ip::tcp::socket socket) {
            if (ec) {
                std::cerr << "error accepting connection: " << ec.message() << std::endl;
                return;
            }
            std::cout << "new connection accepted." << std::endl;
            ws_session_ = WebSocketSession::create(std::move(socket), this);
            ws_session_.run();
        });
    
    ioc_thread_ = std::thread([this]() {
        try {
            ioc_.run();
        } catch (const std::exception& e) {
            std::cerr << "error in io_context thread: " << e.what() << std::endl;
        }
    });
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoHardware::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) {
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoHardware::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/) {
    ioc_.stop();
    if(ioc_.thread_.joinable()) ioc_.thread_.join();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoHardware::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoHardware::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoHardware::on_error(const rclcpp_lifecycle::State& /*previous_state*/) {
    return hardware_interface::CallbackReturn::SUCCESS;
}
std::vector<hardware_interface::StateInterface> MujocoHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for(size_t i = 0; i < qpos_.size(); ++i){
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.at(i).name, hardware_interface::HW_IF_POSITION, &qpos_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.at(i).name, hardware_interface::HW_IF_VELOCITY, &qvel_[i]));
        // state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.at(i).name, hardware_interface::HW_IF_EFFORT, &qtau_[i]));
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
hardware_interface::return_type MujocoHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    // Read state from the last recieived websocket message
    // Why decouple this? The simulation might be paused, and we want to read the last state

    // right now this is optional as state is updated by listening to messages from MuJoCo
    return hardware_interface::return_type::OK;
}
hardware_interface::return_type MujocoHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    // send a websocket message
    for(size_t i = 0; i < cmd_qpos_.size(); ++i) {
        sim_MIT_control(i);
    }
    return hardware_interface::return_type::OK;
}

void MujocoHardware::sim_MIT_control(const int interface_index) const{
    const double qpos_error = cmd_qpos_[interface_index] - qpos_[interface_index];
    const double qvel_error = cmd_qvel_[interface_index] - qvel_[interface_index];
    const double qtau_ff = cmd_qtau_ff_[interface_index];

    const double cmd_torque = KP_ * qpos_error + KD_ * qvel_error + qtau_ff;
    
    std::cout << "torque: "<< cmd_torque << std::endl;
    // TODO: Send cmd_torque to the simulation environment
}

std::shared_ptr<WebSocketSession> WebSocketSession::create(boost::asio::ip::tcp::socket socket, MujocoHardware* hw) 
{
    return std::make_shared<WebSocketSession>(std::move(socket), hw);
}

WebSocketSession::WebSocketSession(boost::asio::ip::tcp::socket socket, MujocoHardware* hw)
    : ws_(std::move(socket)), hw_(hw) {}

void WebSocketSession::run(boost::asio::ip::tcp::socket socket, MujocoHardware* hw){
    const auto session = std::make_shared<WebSocketSession> (std::move(socket), hw);
    session->do_handshake();
}

void WebSocketSession::do_handshake() {
    ws_.set_option(
        boost::beast::websocket::stream_base::timeout::suggested(
            boost::beast::websocket::role_type::server));
    ws_.async_accept(
        boost::beast::bind_front_handler(
            &WebSocketSession::on_accept, shared_from_this()));
}

void WebSocketSession::on_accept(boost::beast::error_code ec){
    if (ec){
        std::cerr << "handshake failed: " << ec.message() << std::endl;
    }
    do_read();
}

void WebSocketSession::do_read(){
    ws_.async_read(buffer_,
    boost::beast::bind_front_handler(&WebSocketSession::on_read, shared_from_this()));
}

void WebSocketSession::on_read(boost::beast::error_code ec, std::size_t bytes_transferred){
    if (ec){
        std::cerr << "read error: " << ec.message() << std::endl;
        return;
    }
    std::string data = boost::beast::buffers_to_string(buffer_.data());
    {
        std::lock_guard<std::mutex> (hw_->state_mutex_);
        try{
            boost::json::value jv = boost::json::parse(data);
            boost::json::object& obj = jv.as_object();
            if(obj.find("key") == obj.end() || obj.find("val") == obj.end()){
                return;
            }
            if(obj["key"].as_string() == "state"){
                boost::json::object& val = obj["val"].as_object();
                boost::json::array& qpos = val["qpos"].as_array();
                boost::json::array& qvel = val["qvel"].as_array();
                for(size_t i = 0; i < qpos.size(); ++i){
                    hw_->qpos_[i] = qpos[i].as_double();
                    hw_->qvel_[i] = qvel[i].as_double();
                }
            }
        }
        catch(const std::exception& e){
            std::cerr << "json parse error: " << std::endl;
        }
    };

    buffer_.consume(bytes_transferred);
    do_read();
}


}; // namespace mujoco_hardware_interface