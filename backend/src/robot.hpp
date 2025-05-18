#ifndef ROBOTIC_ARM_HPP
#define ROBOTIC_ARM_HPP

#include <nlohmann/json.hpp>
#include <vector>

using json = nlohmann::json;

class RobotLink {
public:
    enum class LinkType {
        STATIC,  // Fixed link (no movement)
        X,       // Translation along X
        Y,       // Translation along Y
        Z,       // Translation along Z
        ROT_X,   // Rotation around X
        ROT_Y,   // Rotation around Y
        ROT_Z    // Rotation around Z
    };

    // Constructor
    RobotLink(float x, float y, float z, float rx, float ry, float rz, 
              LinkType type, float minVal, float maxVal, float maxSpeed)
        : translationX(x), translationY(y), translationZ(z),
          rotationX(rx), rotationY(ry), rotationZ(rz),
          type(type), minValue(minVal), maxValue(maxVal), maxSpeed(maxSpeed),
          currentValue(0.0f), requestedValue(0.0f) {}

    // Getters
    float getTranslationX() const { return translationX; }
    float getTranslationY() const { return translationY; }
    float getTranslationZ() const { return translationZ; }
    float getRotationX() const { return rotationX; }
    float getRotationY() const { return rotationY; }
    float getRotationZ() const { return rotationZ; }
    LinkType getLinkType() const { return type; }
    float getCurrentValue() const { return currentValue; }
    float getRequestedValue() const { return requestedValue; }

    // Set requested value (with bounds checking)
    void setRequestedValue(float value) {
        if (value >= minValue && value <= maxValue) {
            requestedValue = value;
        }
    }

    // Temporary function to simulate movement
    void moveAroundZAxis() {
        // Simulate movement (to be replaced with interpolation)
        if (type == LinkType::ROT_Z) {
            currentValue += 0.0174533f; // ~1 deg/sec (1 deg = 0.0174533 rad)
            if (currentValue > 6.2832f) currentValue -= 6.2832f; // Wrap at 2π
        }
    }

    // Get JSON state for this link. Only include relevant values
    json getState() const {
        json state;
        state["translation"] = {{"x", translationX}, {"y", translationY}, {"z", translationZ}};
        state["rotation"] = {{"x", rotationX}, {"y", rotationY}, {"z", rotationZ}};
        return state;
    }

private:
    float translationX, translationY, translationZ; // meters
    float rotationX, rotationY, rotationZ;         // radians
    LinkType type;                                     // Degree of freedom
    float minValue, maxValue;                      // Bounds for DoF (m or rad)
    float maxSpeed;                                // Max speed (m/s or rad/s)
    float currentValue;                            // Current position (m or rad)
    float requestedValue;                          // Requested position (m or rad)
};

class RoboticArm {
public:
    RoboticArm() {
        // Hardcode robot: base (STATIC), arm1 (ROT_Z), arm2 (ROT_Z)
        links = {
            // Base: 1.5m tall, 0.3m x 0.3m
            RobotLink(0.0f, 0.0f, 1.5f, 0.0f, 0.0f, 0.0f, 
                      RobotLink::LinkType::STATIC, 0.0f, 0.0f, 0.0f),
            // Arm1: 1m long (X), rotates around Z at base top (0, 0, 1.5)
            RobotLink(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
                      RobotLink::LinkType::ROT_Z, -3.1416f, 3.1416f, 1.0f), // ±π, 1 rad/s
            // Arm2: 0.7m long (X), rotates around Z at arm1 end
            RobotLink(0.7f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
                      RobotLink::LinkType::ROT_Z, -3.1416f, 3.1416f, 1.0f)  // ±π, 1 rad/s
        };
    }

    void update() {
        for (auto& link : links) {
            link.moveAroundZAxis();
        }
    }

    json getState() const {
        json state = json::array();
        for (const auto& link : links) {
            state.push_back(link.getState());
        }
        return state;
    }

    // Set requested actuator positions
    void setRequestedActuator1(float value) { requestedActuator1 = value; }
    void setRequestedActuator2(float value) { requestedActuator2 = value; }

    // Get requested positions (for future use)
    float getRequestedActuator1() const { return requestedActuator1; }
    float getRequestedActuator2() const { return requestedActuator2; }

private:
    std::vector<RobotLink> links;
    float actuator1;
    float actuator2;
    float requestedActuator1;
    float requestedActuator2;
};

#endif