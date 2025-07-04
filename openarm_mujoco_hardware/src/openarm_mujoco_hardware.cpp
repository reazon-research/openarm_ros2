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

#include <algorithm>
#include <openarm_mujoco_hardware/openarm_mujoco_hardware.hpp>
#include <rclcpp/logging.hpp>

namespace openarm_mujoco_hardware {

hardware_interface::CallbackReturn MujocoHardware::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if (info.hardware_parameters.find("websocket_port") !=
      info.hardware_parameters.end()) {
    const double val = std::stoi(info.hardware_parameters.at("websocket_port"));
    if (val <= 0) {
      return hardware_interface::CallbackReturn::FAILURE;
    }
    websocket_port_ = val;
  } else {
    websocket_port_ = kDefaultWebsocketPort;
  }
  std::cerr << "websocket port: " << websocket_port_ << std::endl;
  address_ = boost::asio::ip::make_address("0.0.0.0");
  endpoint_ = boost::asio::ip::tcp::endpoint(address_, websocket_port_);

  // allocate space for joint states
  const size_t DOF = info_.joints.size();

  qpos_.resize(DOF, 0.0);
  qvel_.resize(DOF, 0.0);
  qtau_.resize(DOF, 0.0);

  cmd_qpos_.resize(DOF, 0.0);
  cmd_qvel_.resize(DOF, 0.0);
  cmd_qtau_ff_.resize(DOF, 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

void MujocoHardware::start_accept() {
  acceptor_.async_accept(
      [this](boost::beast::error_code ec, boost::asio::ip::tcp::socket socket) {
        if (ec) {
          RCLCPP_WARN(rclcpp::get_logger("MujocoHardware"),
                      "accept error: %s — retrying", ec.message().c_str());
          return start_accept();
        }

        if (ws_session_) {
          ws_session_->close_with_delay();
        }

        ws_session_ = WebSocketSession::create(std::move(socket), this);
        ws_session_->run();

        this->start_accept();
      });
}

hardware_interface::CallbackReturn MujocoHardware::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  boost::beast::error_code ec;

  if (acceptor_.is_open()) {
    return hardware_interface::CallbackReturn::FAILURE;
  } else {
    acceptor_.open(endpoint_.protocol(), ec);
    if (ec) {
      throw std::runtime_error("open error: " + ec.message());
    }
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

  start_accept();

  ioc_thread_ = std::thread([this]() {
    try {
      ioc_.run();
    } catch (const std::exception& e) {
      std::cerr << "error in io_context thread: " << e.what() << std::endl;
    }
  });

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoHardware::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoHardware::on_shutdown(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  clear_cmd_torque();
  ioc_.stop();
  if (ioc_thread_.joinable()) ioc_thread_.join();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoHardware::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return hardware_interface::CallbackReturn::SUCCESS;
}

void MujocoHardware::clear_cmd_torque() {
  nlohmann::json cmd_msg;
  nlohmann::json& cmd = cmd_msg["cmd"];
  for (size_t i = 0; i < cmd_qpos_.size(); ++i) {
    cmd[info_.joints[i].name] = 0.0;
  }
  if (ws_session_) {
    ws_session_->send_json(cmd_msg);
  }
}

hardware_interface::CallbackReturn MujocoHardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  clear_cmd_torque();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoHardware::on_error(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  clear_cmd_torque();
  return hardware_interface::CallbackReturn::SUCCESS;
}
std::vector<hardware_interface::StateInterface>
MujocoHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < qpos_.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints.at(i).name, hardware_interface::HW_IF_POSITION,
        &qpos_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints.at(i).name, hardware_interface::HW_IF_VELOCITY,
        &qvel_[i]));
    // state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints.at(i).name,
    // hardware_interface::HW_IF_EFFORT, &qtau_[i]));
  }
  return state_interfaces;
}
std::vector<hardware_interface::CommandInterface>
MujocoHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < qpos_.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints.at(i).name, hardware_interface::HW_IF_POSITION,
        &cmd_qpos_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints.at(i).name, hardware_interface::HW_IF_VELOCITY,
        &cmd_qvel_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints.at(i).name, hardware_interface::HW_IF_EFFORT,
        &cmd_qtau_ff_[i]));
  }
  return command_interfaces;
}
hardware_interface::return_type MujocoHardware::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Read state from the last recieived websocket message
  // Why decouple this? The simulation might be paused, and we want to read the
  // last state

  // right now this is optional as state is updated by listening to messages
  // from MuJoCo
  return hardware_interface::return_type::OK;
}
hardware_interface::return_type MujocoHardware::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // send a websocket message

  nlohmann::json cmd_msg;
  nlohmann::json& cmd = cmd_msg["cmd"];

  for (size_t i = 0; i < cmd_qpos_.size(); ++i) {
    const double qpos_error = cmd_qpos_[i] - qpos_[i];
    const double qvel_error = cmd_qvel_[i] - qvel_[i];
    const double qtau_ff = cmd_qtau_ff_[i];

    double cmd_torque = KP_[i] * qpos_error + KD_[i] * qvel_error + qtau_ff;

    cmd[info_.joints[i].name] = cmd_torque;
  }
  if (ws_session_) {
    ws_session_->send_json(cmd_msg);
  } else {
    std::cerr << "MuJoCo WebSocket session is not active, please connect at "
              << kMuJoCoWebSocketURL_ << std::endl;
  }

  return hardware_interface::return_type::OK;
}

WebSocketSession::WebSocketSession(boost::asio::ip::tcp::socket socket,
                                   MujocoHardware* hw)
    : ws_(std::move(socket)), hw_(hw), write_in_progress_(false) {
  close_timer_ =
      std::make_unique<boost::asio::steady_timer>(ws_.get_executor());
}

std::shared_ptr<WebSocketSession> WebSocketSession::create(
    boost::asio::ip::tcp::socket socket, MujocoHardware* hw) {
  return std::make_shared<WebSocketSession>(std::move(socket), hw);
}

void WebSocketSession::send_json(const nlohmann::json& j) {
  std::shared_ptr<std::string> msg = std::make_shared<std::string>(j.dump());
  boost::asio::post(ws_.get_executor(), [self = shared_from_this(), msg] {
    self->send_queue_.push_back(msg);
    if (!self->write_in_progress_) {
      self->flush();
    }
  });
}

void WebSocketSession::flush() {
  if (send_queue_.empty()) {
    write_in_progress_ = false;
    return;
  }

  write_in_progress_ = true;
  auto msg = send_queue_.front();
  send_queue_.pop_front();

  ws_.async_write(
      boost::asio::buffer(*msg), [self = shared_from_this(), msg](
                                     boost::beast::error_code ec, std::size_t) {
        if (ec) {
          RCLCPP_WARN(rclcpp::get_logger("WebSocketSession"),
                      "write error: %s — triggering reconnect",
                      ec.message().c_str());
          if (self->hw_) {
            self->hw_->start_accept();  // restart accept loop directly
          }
          return;
        }
        self->write_in_progress_ = false;
        self->flush();
      });
}

void WebSocketSession::run() {
  ws_.set_option(boost::beast::websocket::stream_base::timeout::suggested(
      boost::beast::role_type::server));

  ws_.control_callback(
      [self = shared_from_this()](boost::beast::websocket::frame_type kind,
                                  boost::beast::string_view) {
        if (kind == boost::beast::websocket::frame_type::close) return;
        RCLCPP_WARN(rclcpp::get_logger("WebSocketSession"),
                    "Abnormal frame, forcing close");
        self->ws_.next_layer().shutdown(
            boost::asio::ip::tcp::socket::shutdown_both);
      });

  do_handshake();
}

void WebSocketSession::do_handshake() {
  ws_.async_accept(boost::beast::bind_front_handler(
      &WebSocketSession::on_accept, shared_from_this()));
}

void WebSocketSession::on_accept(boost::beast::error_code ec) {
  if (ec) {
    RCLCPP_WARN(rclcpp::get_logger("WebSocketSession"),
                "handshake error: %s — triggering reconnect",
                ec.message().c_str());
    if (hw_) {
      hw_->start_accept();
    }
    return;
  }

  do_read();
}

void WebSocketSession::do_read() {
  ws_.async_read(buffer_, boost::beast::bind_front_handler(
                              &WebSocketSession::on_read, shared_from_this()));
}

void WebSocketSession::on_read(boost::beast::error_code ec,
                               std::size_t bytes_transferred) {
  if (ec) {
    RCLCPP_WARN(rclcpp::get_logger("WebSocketSession"),
                "read error: %s — triggering reconnect", ec.message().c_str());
    if (hw_) {
      hw_->start_accept();
    }
    return;
  }

  std::string data = boost::beast::buffers_to_string(buffer_.data());
  {
    std::lock_guard<std::mutex>(hw_->state_mutex_);
    try {
      nlohmann::json j = nlohmann::json::parse(data);
      // if "state" key exists, update the hardware state
      if (j.contains("state")) {
        const nlohmann::json& state = j["state"];
        for (size_t i = 0; i < hw_->info_.joints.size(); ++i) {
          const auto& joint = hw_->info_.joints[i];
          if (state.contains(joint.name)) {
            const nlohmann::json& joint_data = state.at(joint.name);
            if (joint_data.contains("qpos")) {
              hw_->qpos_[i] = joint_data.at("qpos").get<double>();
            }
            if (joint_data.contains("qvel")) {
              hw_->qvel_[i] = joint_data.at("qvel").get<double>();
            }
          }
        }
      }
    } catch (const nlohmann::json::parse_error& e) {
      std::cerr << "json parse error: " << e.what() << std::endl;
    }
  };

  buffer_.consume(bytes_transferred);
  do_read();
}

void WebSocketSession::close_with_delay() {
  close_timer_->expires_after(kCloseDelayMs);
  close_timer_->async_wait([self = shared_from_this()](
                               boost::beast::error_code ec) {
    if (!ec) {
      boost::beast::error_code close_ec;
      self->ws_.close(boost::beast::websocket::close_code::normal, close_ec);
    }
  });
}
};  // namespace openarm_mujoco_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(openarm_mujoco_hardware::MujocoHardware,
                       hardware_interface::SystemInterface)