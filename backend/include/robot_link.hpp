#ifndef ROBOT_LINK_HPP
#define ROBOT_LINK_HPP

#include <nlohmann/json.hpp>
#include <string>

#include "helpers.hpp"

using json = nlohmann::json;


class PDController
{
public:
    PDController();
    PDController(float Kp, float Kd);

    float compute(float error, float error_deriv) const;

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
        ROT_Z,  // Rotation around Z
        Move_Base // Move base. Make different type so it does not show up in UI
    };

    // Constructor
    RobotLink(const std::string &name, float x, float y, float z, float rx, float ry, float rz,
              LinkType type, float minVal, float maxVal, float maxSpeed, float maxAcc);

    // Getters
    float getTranslationX() const { return translationX; }
    float getTranslationY() const { return translationY; }
    float getTranslationZ() const { return translationZ; }
    float getRotationX() const { return rotationX; }
    float getRotationY() const { return rotationY; }
    float getRotationZ() const { return rotationZ; }
    LinkType getLinkType() const { return type; }
    std::string getLinkName() const { return linkName; }
    float getCurrentPosition() const { return currentPosition; }
    float getRequestedPosition() const { return requestedPosition; }
    float getCurrentVelocity() const { return currentVelocity; }

    // Setters
    void setRequestedPosition(float value);
    void setFeedForwardVelocity(float value);
    void setPose(utils::Pose pose);

    // Control and simulation
    float computePositionControl(float pos_ref, float dt);  // Run the position PD controller.
    float computeVelocityControl(float vel_ref, float dt);  // Run the velocity PD controller. Uses feedforward velocity if set in member variable.
    void simulate(float acc, float dt);                     // Simulate the link's motion based on acceleration and time step.

    // Get JSON state for this link for communication to frontend
    json getState() const;

private:
    std::string linkName;                           // Name of the link
    float translationX, translationY, translationZ; // meters
    float rotationX, rotationY, rotationZ;          // radians
    LinkType type;                                  // Degree of freedom
    float minValue, maxValue;                       // Bounds for DoF (m or rad)
    float maxSpeed;                                 // Max speed (m/s or rad/s)
    float maxAcc;                                   // Max acceleration (m/s² or rad/s²)
    float currentPosition;                          // Current position (m or rad)
    float requestedPosition;                        // Requested position (m or rad)
    float currentVelocity;                          // m/s or rad/s
    float feedForwardVelocity=0;                    // m/s or rad/s
    float prevPosError;                             // Previous position error
    float prevVelError;                             // Previous velocity error
    PDController posController;                     // Position PD controller
    PDController velController;                     // Velocity PD controller

    // Convert Type enum to string
    static std::string toString(LinkType type);
};


#endif