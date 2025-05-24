#ifndef ROBOT_LINK_HPP
#define ROBOT_LINK_HPP

#include <nlohmann/json.hpp>
// #include <vector>
#include <string>
#include <cmath>
#include <iostream>

#include "helpers.hpp"

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
        ROT_Z,  // Rotation around Z
        Move_Base // Move base. Make different type so it does not show up in UI
    };

    // Constructor
    RobotLink(const std::string &name, float x, float y, float z, float rx, float ry, float rz,
              LinkType type, float minVal, float maxVal, float maxSpeed, float maxAcc)
        : linkName(name), translationX(x), translationY(y), translationZ(z),
          rotationX(rx), rotationY(ry), rotationZ(rz),
          type(type), minValue(minVal), maxValue(maxVal), maxSpeed(maxSpeed), maxAcc(maxAcc),
          currentPosition(0.0f), requestedPosition(0.0f), currentVelocity(0.0f),
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
    float getCurrentPosition() const { return currentPosition; }
    float getRequestedPosition() const { return requestedPosition; }
    float getCurrentVelocity() const { return currentVelocity; }

    // Set requested value (with bounds checking)
    void setRequestedPosition(float value)
    {
        if (value >= minValue && value <= maxValue)
        {
            requestedPosition = value;
        }
    }

    // Set feed forward velocity
    void setFeedForwardVelocity(float value)
    {
        feedForwardVelocity = value;
    }

    void setPose(utils::Pose pose)
    {
        translationX = pose.x;
        translationY = pose.y;
        translationZ = pose.z;
        rotationX = pose.rot_x;
        rotationY = pose.rot_y;
        rotationZ = pose.rot_z;
    }

    // Compute acceleration for velocity tracking
    float computeVelocityControl(float vel_ref, float dt) {
        if (type == LinkType::STATIC) return 0.0f;
        float error = vel_ref - currentVelocity;
        float error_deriv = (error - prevVelError) / dt;
        prevVelError = error;
        return velController.compute(error, error_deriv);
    }

    // Compute acceleration for position tracking (cascaded control)
    float computePositionControl(float pos_ref, float dt) {
        if (type == LinkType::STATIC) return 0.0f;
        // Position controller: outputs velocity reference
        float pos_error = pos_ref - currentPosition;
        pos_error = fmod(pos_error + M_PI, 2 * M_PI) - M_PI; // Take the shortest distance

        float pos_error_deriv = (pos_error - prevPosError) / dt;
        // instead of using the derivative of the position error, use the velocity
        // float pos_error_deriv = -currentVelocity; // I think the sign should be negative
        prevPosError = pos_error;
        float vel_ref = posController.compute(pos_error, pos_error_deriv);
        // Add feed forward velocity
        vel_ref += feedForwardVelocity;
        // Limit velocity reference to maxSpeed
        vel_ref = std::max(-maxSpeed, std::min(maxSpeed, vel_ref));
        // Velocity controller: outputs acceleration
        return computeVelocityControl(vel_ref, dt);
    }

    // Simulate one step: update position and velocity based on acceleration
    void simulate(float acc, float dt) {
        if (type == LinkType::STATIC || type == LinkType::Move_Base) return;

        // Apply acceleration limit
        acc = std::max(-maxAcc, std::min(maxAcc, acc));

        // Update velocity
        currentVelocity += acc * dt;
        currentVelocity = std::max(-maxSpeed, std::min(maxSpeed, currentVelocity));

        // Update position
        currentPosition += currentVelocity * dt;

        // For rotational links, wrap position within minValue and maxValue
        if (type == LinkType::ROT_X || type == LinkType::ROT_Y || type == LinkType::ROT_Z) {
            while (currentPosition > maxValue) currentPosition -= 2.0f * M_PI;
            while (currentPosition < minValue) currentPosition += 2.0f * M_PI;
        } else {
            // For translational links, clamp position
            currentPosition = std::max(minValue, std::min(maxValue, currentPosition));
        }

        // Update translation/rotation based on LinkType
        if (type == LinkType::ROT_X) {
            rotationX = currentPosition;
        } else if (type == LinkType::ROT_Y) {
            rotationY = currentPosition;
        } else if (type == LinkType::ROT_Z) {
            rotationZ = currentPosition;
        } else if (type == LinkType::X) {
            translationX = currentPosition;
        } else if (type == LinkType::Y) {
            translationY = currentPosition;
        } else if (type == LinkType::Z) {
            translationZ = currentPosition;
        }
    }

    // Temporary function to simulate movement
    void moveAroundZAxis()
    {
        // Simulate movement (to be replaced with interpolation)
        if (type == LinkType::ROT_Z)
        {
            // currentPosition += 0.0174533f; // ~1 deg/sec (1 deg = 0.0174533 rad)
            // if (currentPosition > 6.2832f) currentPosition -= 6.2832f; // Wrap at 2π
            // rotationZ = currentPosition; // Update rotation
            // instead for now, directly set the value
            rotationZ = requestedPosition; // Update rotation
        }
    }

    void moveToRequestedValue(int updateInterval)
    {
        // TODO: check if requested value is within bounds
        // TODO: make distinction between translation and rotation since rotation can wrap around.
        // move to requested value, with max speed. Use update interval to calculate speed
        float delta = requestedPosition - currentPosition;
        float max_step = maxSpeed * updateInterval / 1000.0f; // convert ms to seconds
        // for debuggin, output delta and max_step
        std::cout << "Delta: " << delta << ", Max Step: " << max_step << std::endl;
        if (std::abs(delta) > max_step)
        {
            if (delta > 0)
            {
                currentPosition += max_step;
            }
            else
            {
                currentPosition -= max_step;
            }
        }
        else
        {
            currentPosition = requestedPosition; // reached requested value
        }
        // write the currentPosition to the correct translation/rotation
        if (type == LinkType::ROT_X)
        {
            rotationX = currentPosition; // Update rotation
        }
        else if (type == LinkType::ROT_Y)
        {
            rotationY = currentPosition; // Update rotation
        }
        else if (type == LinkType::ROT_Z)
        {
            rotationZ = currentPosition; // Update rotation
        }
        else if (type == LinkType::X)
        {
            translationX = currentPosition; // Update translation
        }
        else if (type == LinkType::Y)
        {
            translationY = currentPosition; // Update translation
        }
        else if (type == LinkType::Z)
        {
            translationZ = currentPosition; // Update translation
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
    float currentPosition;                             // Current position (m or rad)
    float requestedPosition;                           // Requested position (m or rad)
    float currentVelocity;                         // m/s or rad/s
    float feedForwardVelocity=0;                     // m/s or rad/s
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
        case LinkType::Move_Base:
            return "Move_Base";
        default:
            return "UNKNOWN";
        }
    }
};


#endif