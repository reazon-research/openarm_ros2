#ifndef MUJOCO_HARDWARE_INTERFACE_HPP_
#define MUJOCO_HARDWARE_INTERFACE_HPP_

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio.hpp>
#include <boost/json.hpp>

#include <iostream>
#include <mutex>

namespace mujoco_hardware_interface {

class WebSocketSession;

class MujocoHardware : public hardware_interface::SystemInterface {
public:
    MujocoHardware() = default;

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& /*info*/) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& /*previous_state*/) override;
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) override;
    hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& /*previous_state*/) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) override;
    hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& /*previous_state*/) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;
    hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

    friend class WebSocketSession;
private:
    static double KP_;
    static double KD_;
    void sim_MIT_control(const int interface_index) const;

    std::vector<double> qpos_;
    std::vector<double> qvel_;
    std::vector<double> qtau_;

    std::vector<double> cmd_qpos_;
    std::vector<double> cmd_qvel_;
    std::vector<double> cmd_qtau_ff_;

    std::mutex state_mutex_;

    // websocket connection to mujoco
    boost::asio::ip::tcp::endpoint endpoint_;
    boost::asio::ip::address address_;
    static constexpr double kWebsocketPort = 1337;
    inline static boost::asio::io_context ioc_;
    static boost::asio::ip::tcp::acceptor acceptor_;
    std::thread ioc_thread_;

    std::shared_ptr<WebSocketSession> ws_session_;

};

class WebSocketSession : public std::enable_shared_from_this<WebSocketSession> {
public:
    static std::shared_ptr<WebSocketSession> create(
            boost::asio::ip::tcp::socket socket,
            MujocoHardware* hw
        );
    void run();
    WebSocketSession(boost::asio::ip::tcp::socket socket, MujocoHardware* hw);
private:
    
    void do_handshake();
    void on_accept(boost::beast::error_code ec);
    void do_read();
    void on_read(boost::beast::error_code ec, std::size_t);

    boost::beast::websocket::stream<boost::asio::ip::tcp::socket> ws_;
    boost::beast::flat_buffer buffer_;
    MujocoHardware* hw_;
};
};

#endif  // MUJOCO_HARDWARE_INTERFACE_HPP_

// namespace net   = boost::asio;            // from <boost/asio.hpp>
// namespace beast = boost::beast;           // from <boost/beast.hpp>
// namespace ws    = beast::websocket;       // from <boost/beast/websocket.hpp>
// namespace json  = boost::json;            // from <boost/json.hpp>
// using tcp       = net::ip::tcp;           // from <boost/asio/ip/tcp.hpp>
