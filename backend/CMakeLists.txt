cmake_minimum_required(VERSION 3.10)
project(RoboticArmBackend)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Boost
find_package(Boost REQUIRED COMPONENTS system thread)

# Find nlohmann/json
find_package(nlohmann_json REQUIRED)

# Include WebSocket++ headers
include_directories(${CMAKE_SOURCE_DIR}/include  ${CMAKE_SOURCE_DIR}/libs/websocketpp)

set(SOURCE_FILES
    src/main.cpp
    # src/robotic_arm.cpp
    src/robot_link.cpp
    src/websocket_server.cpp
    # src/websocket_server.hpp
)

# Build executable
add_executable(backend ${SOURCE_FILES})
target_link_libraries(backend Boost::system Boost::thread nlohmann_json::nlohmann_json)