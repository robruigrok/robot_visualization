#ifndef WEBSOCKET_SERVER_HPP
#define WEBSOCKET_SERVER_HPP

#define ASIO_STANDALONE

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <nlohmann/json.hpp>
#include <set>

#include "robot_arm.hpp"

static_assert(std::is_class_v<websocketpp::config::asio>, "websocketpp::config::asio is not a valid configuration");

using websocketpp::connection_hdl;
using websocketpp::server;
using websocketpp::config::asio;
using json = nlohmann::json;
using ServerType = websocketpp::server<websocketpp::config::asio>;

class WebSocketServer
{
public:
    WebSocketServer(RoboticArm &arm);
    void run(uint16_t port);
    void broadcast(const std::string &message);

private:
    void handleLinkSetpoints(const json& data);
    void handleGoalSetpoints(const json& data);
    void handleMoveBaseSetpoints(const json& data);
    
    ServerType server_;
    std::set<connection_hdl, std::owner_less<connection_hdl>> connections_;
    RoboticArm &arm_;
};

#endif