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

    // Set requested actuator positions
    void setRequestedActuator1(float value) { requestedActuator1 = value; }
    void setRequestedActuator2(float value) { requestedActuator2 = value; }

    // Get requested positions (for future use)
    float getRequestedActuator1() const { return requestedActuator1; }
    float getRequestedActuator2() const { return requestedActuator2; }

private:
    float actuator1;
    float actuator2;
    float requestedActuator1;
    float requestedActuator2;
};

#endif