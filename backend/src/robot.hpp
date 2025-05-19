#ifndef ROBOTIC_ARM_HPP
#define ROBOTIC_ARM_HPP

#include <nlohmann/json.hpp>
#include <vector>
#include <string>

using json = nlohmann::json;

class PDController
{
public:
    PDController() : Kp(0.0f), Kd(0.0f) {}
    PDController(float Kp, float Kd) : Kp(Kp), Kd(Kd) {}

    float compute(float error, float error_deriv) const
    {
        return Kp * error + Kd * error_deriv;
    }

private:
    float Kp; // Proportional gain
    float Kd; // Derivative gain
};

class RobotLink
{
public:
    enum class LinkType
    {
        STATIC, // Fixed link (no movement)
        X,      // Translation along X
        Y,      // Translation along Y
        Z,      // Translation along Z
        ROT_X,  // Rotation around X
        ROT_Y,  // Rotation around Y
        ROT_Z   // Rotation around Z
    };

    // Constructor
    RobotLink(const std::string &name, float x, float y, float z, float rx, float ry, float rz,
              LinkType type, float minVal, float maxVal, float maxSpeed, float maxAcc)
        : linkName(name), translationX(x), translationY(y), translationZ(z),
          rotationX(rx), rotationY(ry), rotationZ(rz),
          type(type), minValue(minVal), maxValue(maxVal), maxSpeed(maxSpeed), maxAcc(maxAcc),
          currentValue(0.0f), requestedValue(0.0f), currentVelocity(0.0f),
          prevPosError(0.0f), prevVelError(0.0f)
    {
        // Initialize PD controllers based on LinkType
        if (type == LinkType::X || type == LinkType::Y || type == LinkType::Z)
        {
            // Translational: meters, m/s, m/s²
            posController = PDController(2.0f, 0.2f); // Kp_pos, Kd_pos
            velController = PDController(5.0f, 0.5f); // Kp_vel, Kd_vel
        }
        else if (type == LinkType::ROT_X || type == LinkType::ROT_Y || type == LinkType::ROT_Z)
        {
            // Rotational: radians, rad/s, rad/s²
            posController = PDController(2.0f, 0.2f);
            velController = PDController(5.0f, 0.5f);
        }
        else
        {
            // STATIC: No control
            posController = PDController(0.0f, 0.0f);
            velController = PDController(0.0f, 0.0f);
        }
    }

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
    float getCurrentVelocity() const { return currentVelocity; }

    // Set requested value (with bounds checking)
    void setRequestedValue(float value)
    {
        if (value >= minValue && value <= maxValue)
        {
            requestedValue = value;
        }
    }

    // Compute acceleration for velocity tracking
    float computeVelocityControl(float vel_ref, float vel_actual, float dt) {
        if (type == LinkType::STATIC) return 0.0f;
        float error = vel_ref - vel_actual;
        float error_deriv = (error - prevVelError) / dt;
        prevVelError = error;
        return velController.compute(error, error_deriv);
    }

    // Compute acceleration for position tracking (cascaded control)
    float computePositionControl(float pos_ref, float pos_actual, float vel_actual, float dt) {
        if (type == LinkType::STATIC) return 0.0f;
        // Position controller: outputs velocity reference
        float pos_error = pos_ref - pos_actual;
        // float pos_error_deriv = (pos_error - prevPosError) / dt;
        // instead of using the derivative of the position error, use the velocity
        float pos_error_deriv = -vel_actual; // I think the sign should be negative
        prevPosError = pos_error;
        float vel_ref = posController.compute(pos_error, pos_error_deriv);
        // Limit velocity reference to maxSpeed
        vel_ref = std::max(-maxSpeed, std::min(maxSpeed, vel_ref));
        // Velocity controller: outputs acceleration
        return computeVelocityControl(vel_ref, vel_actual, dt);
    }

    // Simulate one step: update position and velocity based on acceleration
    void simulate(float acc, float dt) {
        if (type == LinkType::STATIC) return;

        // Apply acceleration limit
        acc = std::max(-maxAcc, std::min(maxAcc, acc));

        // Update velocity
        currentVelocity += acc * dt;
        currentVelocity = std::max(-maxSpeed, std::min(maxSpeed, currentVelocity));

        // Update position
        currentValue += currentVelocity * dt;

        // For rotational links, wrap position within minValue and maxValue
        if (type == LinkType::ROT_X || type == LinkType::ROT_Y || type == LinkType::ROT_Z) {
            while (currentValue > maxValue) currentValue -= 2.0f * M_PI;
            while (currentValue < minValue) currentValue += 2.0f * M_PI;
        } else {
            // For translational links, clamp position
            currentValue = std::max(minValue, std::min(maxValue, currentValue));
        }

        // Update translation/rotation based on LinkType
        if (type == LinkType::ROT_X) {
            rotationX = currentValue;
        } else if (type == LinkType::ROT_Y) {
            rotationY = currentValue;
        } else if (type == LinkType::ROT_Z) {
            rotationZ = currentValue;
        } else if (type == LinkType::X) {
            translationX = currentValue;
        } else if (type == LinkType::Y) {
            translationY = currentValue;
        } else if (type == LinkType::Z) {
            translationZ = currentValue;
        }
    }

    // Temporary function to simulate movement
    void moveAroundZAxis()
    {
        // Simulate movement (to be replaced with interpolation)
        if (type == LinkType::ROT_Z)
        {
            // currentValue += 0.0174533f; // ~1 deg/sec (1 deg = 0.0174533 rad)
            // if (currentValue > 6.2832f) currentValue -= 6.2832f; // Wrap at 2π
            // rotationZ = currentValue; // Update rotation
            // instead for now, directly set the value
            rotationZ = requestedValue; // Update rotation
        }
    }

    void moveToRequestedValue(int updateInterval)
    {
        // TODO: check if requested value is within bounds
        // TODO: make distinction between translation and rotation since rotation can wrap around.
        // move to requested value, with max speed. Use update interval to calculate speed
        float delta = requestedValue - currentValue;
        float max_step = maxSpeed * updateInterval / 1000.0f; // convert ms to seconds
        // for debuggin, output delta and max_step
        std::cout << "Delta: " << delta << ", Max Step: " << max_step << std::endl;
        if (std::abs(delta) > max_step)
        {
            if (delta > 0)
            {
                currentValue += max_step;
            }
            else
            {
                currentValue -= max_step;
            }
        }
        else
        {
            currentValue = requestedValue; // reached requested value
        }
        // write the currentValue to the correct translation/rotation
        if (type == LinkType::ROT_X)
        {
            rotationX = currentValue; // Update rotation
        }
        else if (type == LinkType::ROT_Y)
        {
            rotationY = currentValue; // Update rotation
        }
        else if (type == LinkType::ROT_Z)
        {
            rotationZ = currentValue; // Update rotation
        }
        else if (type == LinkType::X)
        {
            translationX = currentValue; // Update translation
        }
        else if (type == LinkType::Y)
        {
            translationY = currentValue; // Update translation
        }
        else if (type == LinkType::Z)
        {
            translationZ = currentValue; // Update translation
        }
    }

    // Get JSON state for this link. Only include relevant values
    json getState() const
    {
        json state;
        state["link_name"] = linkName;
        state["movable"] = toString(type);
        state["translation"] = {{"x", translationX}, {"y", translationY}, {"z", translationZ}};
        state["rotation"] = {{"x", rotationX}, {"y", rotationY}, {"z", rotationZ}};
        return state;
    }

private:
    std::string linkName;                           // Name of the link (optional)
    float translationX, translationY, translationZ; // meters
    float rotationX, rotationY, rotationZ;          // radians
    LinkType type;                                  // Degree of freedom
    float minValue, maxValue;                       // Bounds for DoF (m or rad)
    float maxSpeed;                                 // Max speed (m/s or rad/s)
    float maxAcc;                                   // Max acceleration (m/s² or rad/s²)
    float currentValue;                             // Current position (m or rad)
    float requestedValue;                           // Requested position (m or rad)
    float currentVelocity;                         // m/s or rad/s
    float prevPosError;                            // Previous position error
    float prevVelError;                            // Previous velocity error
    PDController posController;                    // Position PD controller
    PDController velController;                    // Velocity PD controller

    // Convert Type enum to string
    static std::string toString(LinkType type)
    {
        switch (type)
        {
        case LinkType::STATIC:
            return "STATIC";
        case LinkType::X:
            return "X";
        case LinkType::Y:
            return "Y";
        case LinkType::Z:
            return "Z";
        case LinkType::ROT_X:
            return "ROT_X";
        case LinkType::ROT_Y:
            return "ROT_Y";
        case LinkType::ROT_Z:
            return "ROT_Z";
        default:
            return "UNKNOWN";
        }
    }
};

class RoboticArm
{
public:
    RoboticArm()
    {
        // Hardcode robot: base (STATIC), arm1 (ROT_Z), arm2 (ROT_Z)
        // Entities in links: translation x, y z, rotation x, y, z, type, min, max, speed, acceleration
        links = {
            // Base: 1.5m tall, 0.3m x 0.3m
            RobotLink("base", 0.0f, 0.0f, 1.5f, 0.0f, 0.0f, 0.0f,
                      RobotLink::LinkType::Z, 1.0f, 2.0f, 0.2f, 0.2f),
            // Arm1: 1m long (X), rotates around Z at base top (0, 0, 1.5)
            RobotLink("arm1", 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                      RobotLink::LinkType::ROT_Z, -3.1416f, 3.1416f, 0.2f, 0.2f), // ±π, 1 rad/s
            // Arm offset: 0.2m down (Z), static
            RobotLink("arm_offset", 0.0f, 0.0f, -0.2f, 0.0f, 0.0f, 0.0f,
                      RobotLink::LinkType::STATIC, 0.0f, 0.0f, 0.0f, 0.0f),
            // Arm2: 0.7m long (X), rotates around Z at arm1 end
            RobotLink("arm2", 0.7f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                      RobotLink::LinkType::ROT_Z, -3.1416f, 3.1416f, 0.2f, 0.2f) // ±π, 1 rad/s
        };
    }

    void update()
    {
        float dt = getUpdateInterval() / 1000.0f; // Convert ms to seconds
        for (auto &link : links)
        {
            // link.moveAroundZAxis();
            // link.moveToRequestedValue(getUpdateInterval());
            // call position controller of the link
            float acc = link.computePositionControl(
                link.getRequestedValue(), link.getCurrentValue(), link.getCurrentVelocity(), dt);
            // simulate the link
            link.simulate(acc, dt);
        }
    }

    json getState() const
    {
        json state = json::array();
        for (const auto &link : links)
        {
            state.push_back(link.getState());
        }
        return state;
    }

    // Set requested actuator positions
    // void setRequestedActuator(float value) { requestedActuator1 = value; }

    // Get requested positions (for future use)

    std::vector<RobotLink> &getLinks() { return links; } // TODO: make const, for now like this to set the requested value

    int getUpdateInterval() const { return update_interval_ms; }

private:
    std::vector<RobotLink> links;
    int update_interval_ms = 100; // 100ms update interval
};

#endif