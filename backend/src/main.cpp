// #include <websocketpp/config/asio_no_tls.hpp>
// #include <websocketpp/server.hpp>
// #include <nlohmann/json.hpp>
#include <thread>
// #include <chrono>
// #include <set>
// #include <iostream>

#include "robot_arm.hpp"
#include "websocket_server.hpp"

// using websocketpp::connection_hdl;
// using websocketpp::server;
// using websocketpp::config::asio;
// using json = nlohmann::json;

int main()
{
    RoboticArm arm;
    WebSocketServer ws_server(arm);

    // Update arm state and broadcast every 100ms
    std::thread update_thread([&arm, &ws_server]()
                              {
        while (true) {
            arm.update();
            json state = arm.getState();
            ws_server.broadcast(state.dump());
            std::this_thread::sleep_for(std::chrono::milliseconds(arm.getUpdateInterval()));
        } });

    // Run server on port 3000
    ws_server.run(3000);

    update_thread.join();
    return 0;
}