
#include "robot_link.hpp"
#include <cmath>
#include <iostream>

// PDController implementation
PDController::PDController() : Kp(0.0f), Kd(0.0f) {}
PDController::PDController(float Kp, float Kd) : Kp(Kp), Kd(Kd) {}

float PDController::compute(float error, float error_deriv) const {
    return Kp * error + Kd * error_deriv;   
}

// RobotLink implementation
RobotLink::RobotLink(const std::string &name, float x, float y, float z, float rx, float ry, float rz,
            LinkType type, float minVal, float maxVal, float maxSpeed, float maxAcc)
    : linkName(name), translationX(x), translationY(y), translationZ(z),
        rotationX(rx), rotationY(ry), rotationZ(rz),
        type(type), minValue(minVal), maxValue(maxVal), maxSpeed(maxSpeed), maxAcc(maxAcc),
        currentPosition(0.0f), requestedPosition(0.0f), currentVelocity(0.0f),
        prevPosError(0.0f), prevVelError(0.0f)
{
    // Initialize PD controllers based on LinkType
    if (type == LinkType::X || type == LinkType::Y || type == LinkType::Z) {
        // Translational: meters, m/s, m/s²
        posController = PDController(2.0f, 0.2f); // Kp_pos, Kd_pos
        velController = PDController(5.0f, 0.5f); // Kp_vel, Kd_vel
    } else if (type == LinkType::ROT_X || type == LinkType::ROT_Y || type == LinkType::ROT_Z) {
        // Rotational: radians, rad/s, rad/s²
        posController = PDController(2.0f, 0.2f);   // In the end, I settled for the same gains as translational
        velController = PDController(5.0f, 0.5f);
    } else {
        // STATIC: No control
        posController = PDController(0.0f, 0.0f);
        velController = PDController(0.0f, 0.0f);
    }
}

// Set requested value (with bounds checking)
void RobotLink::setRequestedPosition(float value) {
    if (value >= minValue && value <= maxValue)
    {
        requestedPosition = value;
    }
}

// Set feed forward velocity
void RobotLink::setFeedForwardVelocity(float value) {
    feedForwardVelocity = value;
}

void RobotLink::setPose(utils::Pose pose) {
    translationX = pose.x;
    translationY = pose.y;
    translationZ = pose.z;
    rotationX = pose.rot_x;
    rotationY = pose.rot_y;
    rotationZ = pose.rot_z;
}

utils::Pose RobotLink::getPose() {
    utils::Pose pose;
    pose.x = translationX;
    pose.y = translationY;
    pose.z = translationZ;
    pose.rot_x = rotationX;
    pose.rot_y = rotationY;
    pose.rot_z = rotationZ;
    return pose;
}

// Compute acceleration for velocity tracking
float RobotLink::computeVelocityControl(float vel_ref, float dt) {
    if (type == LinkType::STATIC) return 0.0f;
    float error = vel_ref - currentVelocity;
    float error_deriv = (error - prevVelError) / dt;
    prevVelError = error;
    return velController.compute(error, error_deriv);
}

// Compute acceleration for position tracking (cascaded control)
float RobotLink::computePositionControl(float pos_ref, float dt) {
    if (type == LinkType::STATIC) return 0.0f;
    // Position controller: outputs velocity reference
    float pos_error = pos_ref - currentPosition;
    pos_error = fmod(pos_error + M_PI, 2 * M_PI) - M_PI; // Take the shortest distance

    float pos_error_deriv = (pos_error - prevPosError) / dt;
    // instead of using the derivative of the position error, use the velocity. Only works when not moving the base.
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
void RobotLink::simulate(float acc, float dt) {
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

// Get JSON state for this link. Only include relevant values
json RobotLink::getState() const {
    json state;
    state["link_name"] = linkName;
    state["movable"] = toString(type);
    state["translation"] = {{"x", translationX}, {"y", translationY}, {"z", translationZ}};
    state["rotation"] = {{"x", rotationX}, {"y", rotationY}, {"z", rotationZ}};
    return state;
}

// Convert Type enum to string
std::string RobotLink::toString(LinkType type) {
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
