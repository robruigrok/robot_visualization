#ifndef WEBSOCKET_SERVER_HPP
#define WEBSOCKET_SERVER_HPP

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <nlohmann/json.hpp>
#include <set>
#include <iostream>

#include "robot_arm.hpp"

using websocketpp::connection_hdl;
using websocketpp::server;
using websocketpp::config::asio;
using json = nlohmann::json;

class WebSocketServer
{
public:
    WebSocketServer(RoboticArm &arm) : arm_(arm)
    {
        server_.init_asio();
        server_.set_reuse_addr(true);
        server_.clear_access_channels(websocketpp::log::alevel::all); // Disable logging from websocketpp, which spams a lot
        server_.set_open_handler([this](connection_hdl hdl)
                                 {
            connections_.insert(hdl);
            std::cout << "Client connected" << std::endl; });
        server_.set_close_handler([this](connection_hdl hdl)
                                  {
            connections_.erase(hdl); 
            std::cout << "Client disconnected" << std::endl; });

        server_.set_message_handler([this](connection_hdl hdl, server<asio>::message_ptr msg)
                                    {
            try {
                json data = json::parse(msg->get_payload());
                if (!data.contains("type") || !data.contains("data")) {
                    std::cerr << "Invalid JSON: Missing type or data field" << std::endl;
                    return;
                }

                std::string message_type = data["type"].get<std::string>();

                if (message_type == "link_setpoints") {
                    arm_.setMoveModeSetJoints(true);
                    if (data["data"].is_array()) {
                        // Handle array of {link_name, value}
                        for (const auto& movable_link : data["data"]) {
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
                                        link.setRequestedPosition(value);
                                        break;
                                    }
                                }
                            }
                        }
                    } else {
                    std::cerr << "Invalid JSON: Expected array" << std::endl;
                    }
                } else if (message_type == "goal_setpoints") {
                    arm_.setMoveModeSetJoints(false);
                    if (data["data"].contains("goal_pose")) {
                        const auto& goal_pose = data["data"]["goal_pose"];
                        if (goal_pose.contains("x") && goal_pose.contains("y") && 
                            goal_pose.contains("z") && goal_pose.contains("rotz")) {
                            float x = goal_pose["x"].get<float>();
                            float y = goal_pose["y"].get<float>();
                            float z = goal_pose["z"].get<float>();
                            float rotz = goal_pose["rotz"].get<float>();
                            // Convert rotz from degrees to radians
                            rotz = rotz * M_PI / 180.0f;
                            std::cout << "Received goal_setpoints: x=" << x << ", y=" << y 
                                      << ", z=" << z << ", rotz=" << rotz << " rad" << std::endl;
                            // Set the goal pose in the robotic arm
                            arm_.setGoalPoseWorld(x, y, z, rotz);
                            
                            // TODO: I think this is not required. This function is called in the arm.update() function
                            arm_.moveBase();

                        } else {
                            std::cerr << "Invalid goal_setpoints: Missing x, y, z, or rotz" << std::endl;
                        }
                    } else {
                        std::cerr << "Invalid goal_setpoints: Missing goal_pose" << std::endl;
                    }
                } else if (message_type == "move_base_setpoints") {
                    arm_.setMoveModeSetJoints(false);
                    if (data["data"].contains("goal_pose")) {
                        const auto& goal_pose = data["data"]["goal_pose"];
                        if (goal_pose.contains("x") && goal_pose.contains("y") && 
                            goal_pose.contains("z") && goal_pose.contains("rotz")) {
                            float x = goal_pose["x"].get<float>();
                            float y = goal_pose["y"].get<float>();
                            float z = goal_pose["z"].get<float>();
                            float rotz = goal_pose["rotz"].get<float>();
                            // Convert rotz from degrees to radians
                            rotz = rotz * M_PI / 180.0f;
                            std::cout << "Received move_base_setpoints: x=" << x << ", y=" << y 
                                      << ", z=" << z << ", rotz=" << rotz << " rad" << std::endl;
                            // Set the move_base goal pose in the robotic arm
                            arm_.setMoveBaseGoalPose(x, y, z, rotz);

                            // TODO: I think this is not required. This function is called in the arm.update() function
                            arm_.moveBase();

                        } else {
                            std::cerr << "Invalid move_base_setpoints: Missing x, y, z, or rotz" << std::endl;
                        }
                    }
                    

                } else {
                    std::cerr << "Unknown message type: " << message_type << std::endl;
                }
            } catch (const std::exception& e) {
                std::cerr << "Error parsing JSON: " << e.what() << std::endl;
            } });
    }

    void run(uint16_t port)
    {
        server_.listen(port);
        server_.start_accept();
        server_.run();
    }

    void broadcast(const std::string &message)
    {
        for (const auto &hdl : connections_)
        {
            server_.send(hdl, message, websocketpp::frame::opcode::text);
        }
    }

private:
    server<asio> server_;
    std::set<connection_hdl, std::owner_less<connection_hdl>> connections_;
    RoboticArm &arm_;
};

#endif