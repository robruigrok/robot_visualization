#ifndef ROBOTIC_ARM_HPP
#define ROBOTIC_ARM_HPP

#include <nlohmann/json.hpp>

using json = nlohmann::json;

class RoboticArm {
public:
    RoboticArm() : actuator1(0.0f), actuator2(0.0f) {}

    void update() {
        actuator1 += 1.0f;
        actuator2 += 0.5f;
        if (actuator1 > 360.0f) actuator1 -= 360.0f;
        if (actuator2 > 360.0f) actuator2 -= 360.0f;
    }

    json getState() const {
        return {{"actuator1", actuator1}, {"actuator2", actuator2}};
    }

private:
    float actuator1;
    float actuator2;
};

#endif