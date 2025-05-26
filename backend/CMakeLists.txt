cmake_minimum_required(VERSION 3.10)
project(RoboticArmBackend)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# # Find Boost
# find_package(Boost REQUIRED COMPONENTS system thread)

# # Find nlohmann/json
# find_package(nlohmann_json REQUIRED)

find_package(Threads REQUIRED) # For threading support
find_package(nlohmann_json REQUIRED) # JSON library

# Define ASIO_STANDALONE to use standalone Asio
add_definitions(-DASIO_STANDALONE)

# Include WebSocket++ headers
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/libs/websocketpp
    ${CMAKE_SOURCE_DIR}/libs/asio/asio/include # Adjust path based on submodule location
)

set(SOURCE_FILES
    src/main.cpp
    src/robot_arm.cpp
    src/robot_link.cpp
    src/websocket_server.cpp
)

# Build executable
add_executable(backend ${SOURCE_FILES})
# target_link_libraries(backend Boost::system Boost::thread nlohmann_json::nlohmann_json)
target_link_libraries(backend PRIVATE Threads::Threads nlohmann_json::nlohmann_json)