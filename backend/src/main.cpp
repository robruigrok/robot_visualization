#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <nlohmann/json.hpp>
#include <thread>
#include <chrono>
#include <set>

#include "robot.hpp"

using websocketpp::connection_hdl;
using websocketpp::server;
using websocketpp::config::asio;
using json = nlohmann::json;

class WebSocketServer {
public:
    WebSocketServer() {
        server_.init_asio();
        server_.set_open_handler([this](connection_hdl hdl) {
            connections_.insert(hdl);
        });
        server_.set_close_handler([this](connection_hdl hdl) {
            connections_.erase(hdl); // Fixed: changed 'remove' to 'erase'
        });
    }

    void run(uint16_t port) {
        server_.listen(port);
        server_.start_accept();
        server_.run();
    }

    void broadcast(const std::string& message) {
        for (const auto& hdl : connections_) {
            server_.send(hdl, message, websocketpp::frame::opcode::text);
        }
    }

private:
    server<asio> server_;
    std::set<connection_hdl, std::owner_less<connection_hdl>> connections_;
};

int main() {
    RoboticArm arm;
    WebSocketServer ws_server;

    // Update arm state and broadcast every 100ms
    std::thread update_thread([&arm, &ws_server]() {
        while (true) {
            arm.update();
            json state = arm.getState();
            ws_server.broadcast(state.dump());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });

    // Run server on port 3000
    ws_server.run(3000);

    update_thread.join();
    return 0;
}