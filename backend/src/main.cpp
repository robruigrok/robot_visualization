#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <nlohmann/json.hpp>
#include <thread>
#include <chrono>
#include <set>
#include <iostream>

#include "robot.hpp"

using websocketpp::connection_hdl;
using websocketpp::server;
using websocketpp::config::asio;
using json = nlohmann::json;

class WebSocketServer {
public:
    WebSocketServer(RoboticArm& arm):arm_(arm) {
        server_.init_asio();
        server_.set_reuse_addr(true);
        server_.set_open_handler([this](connection_hdl hdl) {
            connections_.insert(hdl);
            std::cout << "Client connected" << std::endl;
        });
        server_.set_close_handler([this](connection_hdl hdl) {
            connections_.erase(hdl); 
            std::cout << "Client disconnected" << std::endl;
        });
    
        server_.set_message_handler([this](connection_hdl hdl, server<asio>::message_ptr msg) {
            try {
                json data = json::parse(msg->get_payload());
                if (data.is_array()) {
                    // Handle array of {link_name, value}
                    for (const auto& movable_link : data) {
                        if (movable_link.contains("link_name") && movable_link.contains("value")) {
                            std::string linkName = movable_link["link_name"].get<std::string>();
                            float value = movable_link["value"].get<float>();
                            std::cout << "Received " << linkName << ": " << value << " deg/m" << std::endl;
                            // Find the corresponding link and set the requested value
                            std::vector<RobotLink>& links = arm_.getLinks();
                            for (auto& link : links) { // hmmmm this is private. TODO: CHANGE
                                if (link.getLinkName() == linkName) {
                                    // a bit ugly, but if the link is a rotation link, convert to radians
                                    if (link.getLinkType() == RobotLink::LinkType::ROT_X ||
                                        link.getLinkType() == RobotLink::LinkType::ROT_Y ||
                                        link.getLinkType() == RobotLink::LinkType::ROT_Z) {
                                        value = value * M_PI / 180.0f; // convert to radians
                                    }
                                    link.setRequestedValue(value);
                                    break;
                                }
                            }
                        }
                    }
                } else {
                    std::cerr << "Invalid JSON: Expected array" << std::endl;
                }
            } catch (const std::exception& e) {
                std::cerr << "Error parsing JSON: " << e.what() << std::endl;
            }
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
    RoboticArm& arm_;
};

int main() {
    RoboticArm arm;
    WebSocketServer ws_server(arm);

    // Update arm state and broadcast every 100ms
    std::thread update_thread([&arm, &ws_server]() {
        while (true) {
            arm.update();
            json state = arm.getState();
            ws_server.broadcast(state.dump());
            std::this_thread::sleep_for(std::chrono::milliseconds(arm.getUpdateInterval()));
        }
    });

    // Run server on port 3000
    ws_server.run(3000);

    update_thread.join();
    return 0;
}