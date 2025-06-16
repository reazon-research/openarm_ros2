#include <boost/asio.hpp>           // for boost::asio::ip::tcp, io_context
#include <boost/beast/core.hpp>      // for beast::flat_buffer
#include <boost/beast/websocket.hpp> // for beast::websocket::stream
#include <boost/json.hpp>            // for boost::json::value, parse, object
#include <iostream>                  // for std::cerr, std::cout
#include <memory>                    // for std::make_shared
#include <string>      

namespace net   = boost::asio;            // from <boost/asio.hpp>
namespace beast = boost::beast;           // from <boost/beast.hpp>
namespace ws    = beast::websocket;       // from <boost/beast/websocket.hpp>
namespace json  = boost::json;            // from <boost/json.hpp>
using tcp       = net::ip::tcp;           // from <boost/asio/ip/tcp.hpp>


class WebSocketSession : public std::enable_shared_from_this<WebSocketSession> {
public:
    explicit WebSocketSession(tcp::socket socket)
        : ws_(std::move(socket))
    {}

    // Start the asynchronous handshake and read loop
    void run() {
        // Set suggested timeout settings for the websocket
        ws_.set_option(ws::stream_base::timeout::suggested(beast::role_type::server));

        // Accept the websocket handshake
        ws_.async_accept(
            beast::bind_front_handler(&WebSocketSession::on_accept, shared_from_this()));
    }

private:
    ws::stream<tcp::socket> ws_;
    beast::flat_buffer buffer_;

    void on_accept(beast::error_code ec) {
        if (ec) {
            std::cerr << "Accept error: " << ec.message() << "\n";
            return;
        }
        read_message();
    }

    void read_message() {
        // Clear buffer before reading
        buffer_.consume(buffer_.size());

        ws_.async_read(
            buffer_,
            beast::bind_front_handler(&WebSocketSession::on_read, shared_from_this()));
    }

    void on_read(beast::error_code ec, std::size_t bytes_transferred) {
        boost::ignore_unused(bytes_transferred);

        if (ec == ws::error::closed) {
            // Connection closed gracefully
            return;
        }
        if (ec) {
            std::cerr << "Read error: " << ec.message() << "\n";
            return;
        }

        // Convert buffer to string
        std::string msg = beast::buffers_to_string(buffer_.data());

        try {
            // Parse JSON
            json::value jv = json::parse(msg);
            if (!jv.is_object()) {
                std::cerr << "Received JSON is not an object\n";
            } else {
                json::object obj = jv.as_object();

                // Check for "key" and "val"
                if (!obj.contains("key")) {
                    std::cerr << "No key specified in the message.\n";
                } else if (!obj.contains("val")) {
                    std::cerr << "No val specified in the message.\n";
                } else {
                    std::string key = json::value_to<std::string>(obj["key"]);
                    json::value val = obj["val"];

                    if (key == "state" && val.is_object()) {
                        g_state = val.as_object();
                    }

                    std::cout << "Received key: " << key
                              << ", val: " << json::serialize(val) << std::endl;
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "JSON parse error: " << e.what() << "\n";
        }

        // Continue reading
        read_message();
    }
};

class WebSocketListener : public std::enable_shared_from_this<WebSocketListener> {
public:
    WebSocketListener(net::io_context& ioc, tcp::endpoint endpoint)
        : ioc_(ioc)
        , acceptor_(ioc)
    {
        beast::error_code ec;

        // Open the acceptor
        acceptor_.open(endpoint.protocol(), ec);
        if (ec) {
            throw std::runtime_error("Open error: " + ec.message());
        }

        // Allow address reuse
        acceptor_.set_option(net::socket_base::reuse_address(true), ec);
        if (ec) {
            throw std::runtime_error("Set_option error: " + ec.message());
        }

        // Bind to the server address
        acceptor_.bind(endpoint, ec);
        if (ec) {
            throw std::runtime_error("Bind error: " + ec.message());
        }

        // Start listening for connections
        acceptor_.listen(net::socket_base::max_listen_connections, ec);
        if (ec) {
            throw std::runtime_error("Listen error: " + ec.message());
        }
    }

    void run() {
        do_accept();
    }

private:
    net::io_context& ioc_;
    tcp::acceptor acceptor_;

    void do_accept() {
        acceptor_.async_accept(
            net::make_strand(ioc_),
            beast::bind_front_handler(&WebSocketListener::on_accept, shared_from_this()));
    }

    void on_accept(beast::error_code ec, tcp::socket socket) {
        if (ec) {
            std::cerr << "Accept error: " << ec.message() << "\n";
        } else {
            // Launch a new session for this connection
            std::make_shared<WebSocketSession>(std::move(socket))->run();
        }
        do_accept();
    }
};
