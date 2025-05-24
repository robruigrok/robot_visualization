#pragma once

#include <cmath>


namespace utils {
    struct Pose
    {
        float x = 0.0f, y = 0.0f, z = 0.0f; // meters
        float rot_x = 0.0f, rot_y = 0.0f, rot_z = 0.0f; // radians
    };

    struct Velocity
    {
        float vel_x = 0.0f, vel_y = 0.0f, vel_z = 0.0f; // m/s
        float rot_x = 0.0f, rot_y = 0.0f, rot_z = 0.0f; // rad/s
    };
    struct MoveBase
    {
        Pose pose;
        Velocity velocity;
    };

    float getAngleDifference(float angle1, float angle2)
    {
        float diff = angle2 - angle1;
        return std::remainder(diff, 2.0f * M_PI);
    }
}