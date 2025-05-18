#ifndef ROBOTIC_ARM_HPP
#define ROBOTIC_ARM_HPP

#include <nlohmann/json.hpp>
#include <vector>
#include <string>


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
    RobotLink(const std::string& name, float x, float y, float z, float rx, float ry, float rz, 
              LinkType type, float minVal, float maxVal, float maxSpeed)
        : linkName(name), translationX(x), translationY(y), translationZ(z),
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
    std::string getLinkName() const { return linkName; }
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
            // currentValue += 0.0174533f; // ~1 deg/sec (1 deg = 0.0174533 rad)
            // if (currentValue > 6.2832f) currentValue -= 6.2832f; // Wrap at 2π
            // rotationZ = currentValue; // Update rotation
            // instead for now, directly set the value
            rotationZ = requestedValue; // Update rotation
        }
    }

    void moveToRequestedValue(int updateInterval) {
        // TODO: check if requested value is within bounds
        // TODO: make distinction between translation and rotation since rotation can wrap around.
        // move to requested value, with max speed. Use update interval to calculate speed
        float delta = requestedValue - currentValue;
        float max_step = maxSpeed * updateInterval / 1000.0f; // convert ms to seconds
        // for debuggin, output delta and max_step
        std::cout << "Delta: " << delta << ", Max Step: " << max_step << std::endl;
        if (std::abs(delta) > max_step) {
            if (delta > 0) {
                currentValue += max_step;
            } else {
                currentValue -= max_step;
            }
        } else {
            currentValue = requestedValue; // reached requested value
        }
        // write the currentValue to the correct translation/rotation
        if (type == LinkType::ROT_X) {
            rotationX = currentValue; // Update rotation    
        } else if (type == LinkType::ROT_Y) {
            rotationY = currentValue; // Update rotation
        } else if (type == LinkType::ROT_Z) {
            rotationZ = currentValue; // Update rotation
        } else if (type == LinkType::X) {
            translationX = currentValue; // Update translation
        } else if (type == LinkType::Y) {
            translationY = currentValue; // Update translation
        } else if (type == LinkType::Z) {
            translationZ = currentValue; // Update translation
        }
    }

    // Get JSON state for this link. Only include relevant values
    json getState() const {
        json state;
        state["link_name"] = linkName;
        state["movable"] = toString(type);
        state["translation"] = {{"x", translationX}, {"y", translationY}, {"z", translationZ}};
        state["rotation"] = {{"x", rotationX}, {"y", rotationY}, {"z", rotationZ}};
        return state;
    }

private:
    std::string linkName; // Name of the link (optional) 
    float translationX, translationY, translationZ; // meters
    float rotationX, rotationY, rotationZ;         // radians
    LinkType type;                                     // Degree of freedom
    float minValue, maxValue;                      // Bounds for DoF (m or rad)
    float maxSpeed;                                // Max speed (m/s or rad/s)
    float currentValue;                            // Current position (m or rad)
    float requestedValue;                          // Requested position (m or rad)

    // Convert Type enum to string
    static std::string toString(LinkType type) {
        switch (type) {
            case LinkType::STATIC: return "STATIC";
            case LinkType::X: return "X";
            case LinkType::Y: return "Y";
            case LinkType::Z: return "Z";
            case LinkType::ROT_X: return "ROT_X";
            case LinkType::ROT_Y: return "ROT_Y";
            case LinkType::ROT_Z: return "ROT_Z";
            default: return "UNKNOWN";
        }
    }

};

class RoboticArm {
public:
    RoboticArm() {
        // Hardcode robot: base (STATIC), arm1 (ROT_Z), arm2 (ROT_Z)
        // Entities in links: translation x, y z, rotation x, y, z, type, min, max, speed
        links = {
            // Base: 1.5m tall, 0.3m x 0.3m
            RobotLink("base", 0.0f, 0.0f, 1.5f, 0.0f, 0.0f, 0.0f, 
                      RobotLink::LinkType::Z, 1.0f, 2.0f, 0.5f),
            // Arm1: 1m long (X), rotates around Z at base top (0, 0, 1.5)
            RobotLink("arm1", 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
                      RobotLink::LinkType::ROT_Z, -3.1416f, 3.1416f, 0.1f), // ±π, 1 rad/s
            // Arm offset: 0.2m down (Z), static
            RobotLink("arm_offset", 0.0f, 0.0f, -0.2f, 0.0f, 0.0f, 0.0f, 
                      RobotLink::LinkType::STATIC, 0.0f, 0.0f, 0.0f),
            // Arm2: 0.7m long (X), rotates around Z at arm1 end
            RobotLink("arm2", 0.7f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 
                      RobotLink::LinkType::ROT_Z, -3.1416f, 3.1416f, 0.1f)  // ±π, 1 rad/s
        };
    }

    void update() {
        for (auto& link : links) {
            // link.moveAroundZAxis();
            link.moveToRequestedValue(getUpdateInterval()); 
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
    // void setRequestedActuator(float value) { requestedActuator1 = value; }

    // Get requested positions (for future use)

    std::vector<RobotLink>& getLinks() { return links; }    // TODO: make const, for now like this to set the requested value

    int getUpdateInterval() const { return update_interval_ms; }

private:
    std::vector<RobotLink> links;
    int update_interval_ms = 100;    // 100ms update interval
};

#endif