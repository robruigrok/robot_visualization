#ifndef ROBOTIC_ARM_HPP
#define ROBOTIC_ARM_HPP

#include <nlohmann/json.hpp>
#include <vector>
#include <string>

#include "helpers.hpp"
#include "robot_link.hpp"

using json = nlohmann::json;

class RoboticArm
{
public:
    RoboticArm(); // The constructor defines the shape of the robot arm. It initializes the links with their properties.

    // simulate the links and base one step
    void update();
    void moveBase();

    // Getters
    std::tuple<float, float, float> getLinkLengths(const std::string& arm1, const std::string& arm2, const std::string& arm3) const;
    std::tuple<float, float, float> getCurrentAngles(const std::string& arm1, const std::string& arm2, const std::string& arm3) const;
    std::tuple<float, float, float> getLinkGoalPositions(const std::string& arm1, const std::string& arm2, const std::string& arm3) const;
    json getState() const;  // return the state of the robot arm as a JSON object for the frontend
    std::vector<RobotLink> &getLinks();
    int getUpdateInterval() const;

    // Setters
    void setGoalPose(float x, float y, float z, float rotz);
    void setGoalPoseWorld(float x, float y, float z, float rotz);
    void setMoveBaseGoalPose(float x, float y, float z, float rotz);
    void setFeedForwardVelocity(const std::string& arm1, float ff1, const std::string& arm2, float ff2, const std::string& arm3, float ff3, const std::string& base, float ffz) ;
    void setRequestedAngles(float rot1, float rot2, float rot3);
    void setMoveModeSetJoints(bool mode);

private:
    // Conversions and computations
    utils::Pose convertGoalPoseInBaseFrame(utils::Pose goal_pose);
    void computeJointAngles(float x, float y, float rot_z);
    void computeZMotion();
    void computeMoveBaseVelocity();
    struct JointAngles {
        float rot1, rot2, rot3;
    };

    std::vector<RobotLink> links;
    int update_interval_ms = 100;       // 100ms update interval
    // store the goal pose (robot frame)
    float goal_x, goal_y, goal_z;       // meters
    float goal_rot_z;                   // radians
    // store goal pose (world frame)
    utils::Pose goal_pose_world = {};
    // set current position of base
    utils::MoveBase move_base = {};     // position and rotation of the base
    utils::Pose move_base_goal = {};    // goal position and rotation of the base. Ignore velocity.
    float move_base_velocity = 0.2f;    // m/s
    float move_base_rotation = 0.2f;    // rad/s
    bool move_mode_set_joints = true;   // true if move mode is set to joints, false if set to base  
};

#endif