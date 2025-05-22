#ifndef ROBOTIC_ARM_HPP
#define ROBOTIC_ARM_HPP

#include <nlohmann/json.hpp>
#include <vector>
#include <string>
#include <cmath>

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
          currentPosition(0.0f), requestedValue(0.0f), currentVelocity(0.0f),
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

    void setPose(Pose pose)
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
        // Limit velocity reference to maxSpeed
        vel_ref = std::max(-maxSpeed, std::min(maxSpeed, vel_ref));
        // Velocity controller: outputs acceleration
        return computeVelocityControl(vel_ref, dt);
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
            rotationZ = requestedValue; // Update rotation
        }
    }

    void moveToRequestedValue(int updateInterval)
    {
        // TODO: check if requested value is within bounds
        // TODO: make distinction between translation and rotation since rotation can wrap around.
        // move to requested value, with max speed. Use update interval to calculate speed
        float delta = requestedValue - currentPosition;
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
            currentPosition = requestedValue; // reached requested value
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
        case LinkType::Move_Base:
            return "Move_Base";
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
        float velocity_gain = 2.0f;
        float acceleration_gain = 2.0f;
        float range_gain = 4.0f;
        links = {
            RobotLink("move_base_x", 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                      RobotLink::LinkType::Move_Base, 0.0, 0.0f, 0.0f, 0.0f),     
            // Base: 1.5m tall, 0.3m x 0.3m
            RobotLink("base", 0.0f, 0.0f, 1.5f, 0.0f, 0.0f, 0.0f,
                      RobotLink::LinkType::Z, 1.0f, 2.0f, 0.2f, 0.2f),
            // Arm1: 1m long (X), rotates around Z at base top (0, 0, 1.5)
            RobotLink("arm1", 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                      RobotLink::LinkType::ROT_Z, range_gain*-3.1416f, range_gain*3.1416f, velocity_gain*0.2f, acceleration_gain*0.2f), // ±π, 1 rad/s
            // Arm offset: 0.2m down (Z), static
            RobotLink("arm_offset", 0.0f, 0.0f, -0.2f, 0.0f, 0.0f, 0.0f,
                      RobotLink::LinkType::STATIC, 0.0f, 0.0f, 0.0f, 0.0f),
            // Arm2: 0.7m long (X), rotates around Z at arm1 end
            RobotLink("arm2", 0.7f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                      RobotLink::LinkType::ROT_Z, range_gain*-3.1416f, range_gain*3.1416f, velocity_gain*0.2f, acceleration_gain*0.2f), // ±π, 1 rad/s
            // Arm offset: 0.2m down (Z), static
            RobotLink("arm_offset", 0.0f, 0.0f, -0.2f, 0.0f, 0.0f, 0.0f,
                      RobotLink::LinkType::STATIC, 0.0f, 0.0f, 0.0f, 0.0f),        
                      // Arm3: 0.4m long (X), rotates around Z at arm1 end
            RobotLink("arm3", 0.4f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                      RobotLink::LinkType::ROT_Z, range_gain*-3.1416f, range_gain*3.1416f, velocity_gain*0.2f, acceleration_gain*0.2f) 
        };
    }

    void update()
    {
        float dt = getUpdateInterval() / 1000.0f; // Convert ms to seconds
        moveBase(); // move the base
        for (auto &link : links)
        {
            // link.moveAroundZAxis();
            // link.moveToRequestedValue(getUpdateInterval());
            // call position controller of the link
            float acc = link.computePositionControl(
                link.getRequestedValue(), dt);
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

    // Set goal pose
    void setGoalPose(float x, float y, float z, float rotz)
    {
        goal_x = x;
        goal_y = y;
        goal_z = z;
        goal_rot_z = rotz; // should already be in radians
    }

    void setGoalPoseWorld(float x, float y, float z, float rotz)
    {
        goal_pose_world.x = x;
        goal_pose_world.y = y;
        goal_pose_world.z = z;
        goal_pose_world.rot_z = rotz; // should already be in radians
    }

    void setMoveBaseGoalPose(float x, float y, float z, float rotz)
    {
        move_base_goal.x = x;
        move_base_goal.y = y;
        move_base_goal.z = z;
        move_base_goal.rot_z = rotz; // should already be in radians
    }

    void moveBase()
    {
        // move_base.pose = move_base_goal; // for testing
        std::cout << "test" << std::endl;   
        computeMoveBaseVelocity();  // compute the velocity of the base
        // move the base one step.
        for (auto &link : links)
        {
            if (link.getLinkName() == "move_base_x")
            {   
                Pose new_pose = move_base.pose;
                new_pose.x += move_base.velocity.vel_x * getUpdateInterval() / 1000.0f;
                new_pose.y += move_base.velocity.vel_y * getUpdateInterval() / 1000.0f;
                new_pose.z += move_base.velocity.vel_z * getUpdateInterval() / 1000.0f;
                new_pose.rot_z += move_base.velocity.rot_z * getUpdateInterval() / 1000.0f;
                // update
                move_base.pose = new_pose;
                link.setPose(new_pose); // for frontend
            }
        }
        
        // step 1: express the goal position in the base frame
        // Pose goal_pose_world; // ugly, use Pose object perhaps later.
        // goal_pose_world.x = goal_x;
        // goal_pose_world.y = goal_y;
        // goal_pose_world.z = goal_z;
        // goal_pose_world.rot_z = goal_rot_z;
        Pose goal_pose_wrt_robot = convertGoalPoseInBaseFrame(goal_pose_world);

        // print
        std::cout << "Goal pose in base frame: x=" << goal_pose_wrt_robot.x
                  << ", y=" << goal_pose_wrt_robot.y
                  << ", z=" << goal_pose_wrt_robot.z
                  << ", rot_z=" << goal_pose_wrt_robot.rot_z << std::endl;

        // step 2: with this position, call the computeJointAngles function
        setGoalPose(goal_pose_wrt_robot.x, goal_pose_wrt_robot.y, goal_pose_wrt_robot.z, goal_pose_wrt_robot.rot_z);

        // compute and set new joint reference angles
        computeJointAngles(goal_pose_wrt_robot.x, goal_pose_wrt_robot.y, goal_pose_wrt_robot.rot_z);
        // optional step 2.5: get base velocity and compute velocity feed foward for joints.

        // step 3: move the base and check again next time step.
        
    }

    Pose convertGoalPoseInBaseFrame(Pose goal_pose)
    {
        // Get base pose in world frame
        float x_r = move_base.pose.x;
        float y_r = move_base.pose.y;
        float z_r = move_base.pose.z;
        float rot_z_r = move_base.pose.rot_z;

        // translat the goal pose
        float x_trans = goal_pose.x - x_r;
        float y_trans = goal_pose.y - y_r;
        float z_trans = goal_pose.z - z_r;

        // rotate base
        float cos_theta = cos(-rot_z_r);
        float sin_theta = sin(-rot_z_r);
        
        Pose goal_pose_wrt_robot;
        goal_pose_wrt_robot.x = x_trans * cos_theta - y_trans * sin_theta;
        goal_pose_wrt_robot.y = x_trans * sin_theta + y_trans * cos_theta;
        goal_pose_wrt_robot.z = z_trans;
        goal_pose_wrt_robot.rot_z = goal_pose.rot_z - rot_z_r; // Relative orientation    

        return goal_pose_wrt_robot;
    }


    // Getter specifically for Monumental Robot with 3 arms
    // void getLinkLengths(float& arm1_length, float& arm2_length, float& arm3_length,
    //                                 const std::string& arm1, const std::string& arm2, const std::string& arm3) const {

    //     for (const auto& link : links) {
    //         const std::string& name = link.getLinkName();
    //         if (name == arm1) {
    //             arm1_length = link.getTranslationX();
    //         } else if (name == arm2) {
    //             arm2_length = link.getTranslationX();
    //         } else if (name == arm3) {
    //             arm3_length = link.getTranslationX();
    //         }
    //     }
    // }

    std::tuple<float, float, float> getLinkLengths(const std::string& arm1, const std::string& arm2, const std::string& arm3) const {
        float arm1_length = 0.0f, arm2_length = 0.0f, arm3_length = 0.0f;
        for (const auto& link : links) {
            const std::string& name = link.getLinkName();
            if (name == arm1) {
                arm1_length = link.getTranslationX();
            } else if (name == arm2) {
                arm2_length = link.getTranslationX();
            } else if (name == arm3) {
                arm3_length = link.getTranslationX();
            }
        }
        return {arm1_length, arm2_length, arm3_length};
    }

    std::tuple<float, float, float> getCurrentAngles(const std::string& arm1, const std::string& arm2, const std::string& arm3) const {
    float rot1 = 0.0f, rot2 = 0.0f, rot3 = 0.0f;
        for ( auto& link : links) {
            const std::string& name = link.getLinkName();
            if (name == arm1) {
                rot1 = link.getRotationZ();
            } else if (name == arm2) {
                rot2 = link.getRotationZ();
            } else if (name == arm3) {
                rot3 = link.getRotationZ();
            }
        }
        return {rot1, rot2, rot3};
    }

    void setRequestedAngles(float rot1, float rot2, float rot3) {
        for (auto& link : links) {
            const std::string& name = link.getLinkName();
            if (name == "arm1") {
                link.setRequestedValue(rot1);
            } else if (name == "arm2") {
                link.setRequestedValue(rot2);
            } else if (name == "arm3") {
                link.setRequestedValue(rot3);
            }
        }
    }

    struct JointAngles {
        float rot1, rot2, rot3;
    };

    // JointAngles computeJointAngles(float x, float y, float rot_z, const std::vector<float>& current_angles) const;

    void computeJointAngles(float x, float y, float rot_z) {
        // Get arm lengths
        auto [L1, L2, L3] = getLinkLengths("arm1", "arm2", "arm3");
        // Get current angles
        auto [current_rot1, current_rot2, current_rot3] = getCurrentAngles("arm1", "arm2", "arm3");

        // Compute position of joint between arm2 and arm3
        float x2 = x - L3 * std::cos(rot_z);
        float y2 = y - L3 * std::sin(rot_z);

        // Compute distance to (x2, y2)
        float r = std::sqrt(x2 * x2 + y2 * y2);

        // Check reachability
        if (r > L1 + L2 || r < std::abs(L1 - L2)) {
            std::cerr << "Error: Target position (" << x << ", " << y << ") is unreachable" << std::endl;
            return;
        }

        // Compute rot2 using law of cosines
        float cos_rot2 = (r * r - L1 * L1 - L2 * L2) / (2 * L1 * L2);
        // Clamp to [-1, 1] to handle numerical errors
        cos_rot2 = std::max(-1.0f, std::min(1.0f, cos_rot2));
        float rot2_pos = std::acos(cos_rot2);
        float rot2_neg = -rot2_pos;

        // Compute rot1 for both solutions
        float beta = std::atan2(y2, x2);
        std::vector<JointAngles> solutions(2);

        // Solution 1: Positive rot2
        float alpha_pos = std::atan2(L2 * std::sin(rot2_pos), L1 + L2 * std::cos(rot2_pos));
        solutions[0].rot1 = beta - alpha_pos;
        solutions[0].rot2 = rot2_pos;
        solutions[0].rot3 = rot_z - solutions[0].rot1 - solutions[0].rot2;

        // Solution 2: Negative rot2
        float alpha_neg = std::atan2(L2 * std::sin(rot2_neg), L1 + L2 * std::cos(rot2_neg));
        solutions[1].rot1 = beta - alpha_neg;
        solutions[1].rot2 = rot2_neg;
        solutions[1].rot3 = rot_z - solutions[1].rot1 - solutions[1].rot2;
        
        // Choose the solution closest to the current angles
        float min_distance = std::numeric_limits<float>::max();
        int best_solution = 0;
        for (int i = 0; i < 2; ++i) {
            float distance = std::pow(solutions[i].rot1 - current_rot1, 2) +
                            std::pow(solutions[i].rot2 - current_rot2, 2) +
                            std::pow(solutions[i].rot3 - current_rot3, 2);
            if (distance < min_distance) {
                min_distance = distance;
                best_solution = i;
            }
        }

        // return solutions[best_solution];
        setRequestedAngles(solutions[best_solution].rot1, solutions[best_solution].rot2, solutions[best_solution].rot3);
        std::cout << "Computed joint angles: " << solutions[best_solution].rot1 << ", "
                  << solutions[best_solution].rot2 << ", " << solutions[best_solution].rot3 << std::endl;
        
    }

    void computeMoveBaseVelocity()
    {
        // print some debug info
        std::cout << "Top of computeMoveBaseVelocity. Something is wrong." << std::endl;
        
        // Step 1: Compute translation differences
        float dx = move_base_goal.x - move_base.pose.x;
        float dy = move_base_goal.y - move_base.pose.y;
        float dz = move_base_goal.z - move_base.pose.z;

        // Step 2: Compute shortest angular difference (handles wrapping)
        float angular_error = move_base_goal.rot_z - move_base.pose.rot_z;
        angular_error = fmod(angular_error + M_PI, 2 * M_PI) - M_PI; // Normalize to [-pi, pi] TODO FIND SHORTEST DISTANCE

        // Step 3: Compute linear distance (Euclidean distance in 3D)
        float linear_distance = sqrt(dx * dx + dy * dy + dz * dz);

        // Step 4: Avoid division by zero and check if already at goal
        if (linear_distance < 1e-6 && fabs(angular_error) < 1e-6) {
            // No movement needed
            move_base.velocity.vel_x = 0.0f;
            move_base.velocity.vel_y = 0.0f;
            move_base.velocity.vel_z = 0.0f;
            move_base.velocity.rot_z = 0.0f;
            return;
        }

        // Step 5: Compute time to complete motion based on max linear speed
        float time_to_complete = linear_distance / move_base_velocity;

        // Step 6: Compute linear velocities
        if (time_to_complete < update_interval_ms/1000.0f) {
            move_base.velocity.vel_x = dx / (update_interval_ms/1000.0f);
            move_base.velocity.vel_y = dy / (update_interval_ms/1000.0f);
            move_base.velocity.vel_z = dz / (update_interval_ms/1000.0f);
        } else if (linear_distance > 1e-6) {
            move_base.velocity.vel_x = dx / time_to_complete;
            move_base.velocity.vel_y = dy / time_to_complete;
            move_base.velocity.vel_z = dz / time_to_complete;
        }

        // Step 7: Compute angular velocity to finish rotation at the same time
        if (time_to_complete > 1e-6) {
            move_base.velocity.rot_z = angular_error / time_to_complete;
        } else {
            // If linear distance is zero, use max angular speed to rotate in place
            move_base.velocity.rot_z = angular_error > 0 ? move_base_rotation : -move_base_rotation;
            if (fabs(angular_error) < 1e-6) {
                move_base.velocity.rot_z = 0.0f;
            }
        }

        // Step 8: Limit angular velocity if it exceeds move_base_rotation
        if (fabs(move_base.velocity.rot_z) > move_base_rotation) {
            float scale = move_base_rotation / fabs(move_base.velocity.rot_z);
            move_base.velocity.rot_z *= scale;
            // Scale linear velocities to maintain synchronized arrival
            move_base.velocity.vel_x *= scale;
            move_base.velocity.vel_y *= scale;
            move_base.velocity.vel_z *= scale;
        }

        // print the computed velocities
        std::cout << "Computed move_base velocities: "
                  << "vel_x=" << move_base.velocity.vel_x
                  << ", vel_y=" << move_base.velocity.vel_y
                  << ", vel_z=" << move_base.velocity.vel_z
                  << ", rot_z=" << move_base.velocity.rot_z << std::endl;
    }


    // Set requested actuator positions
    // void setRequestedActuator(float value) { requestedActuator1 = value; }

    // Get requested positions (for future use)

    std::vector<RobotLink> &getLinks() { return links; } // TODO: make const, for now like this to set the requested value

    int getUpdateInterval() const { return update_interval_ms; }

private:
    std::vector<RobotLink> links;
    int update_interval_ms = 100; // 100ms update interval
    // store the goal pose (robot frame)
    float goal_x, goal_y, goal_z;   // meters
    float goal_rot_z;                // radians
    // store goal pose (world frame)
    Pose goal_pose_world = {};
    // set current position ov base
    MoveBase move_base = {};        // position and rotation of the base
    Pose move_base_goal = {};   // goal position and rotation of the base. Ignore velocity.
    float move_base_velocity = 0.2f; // m/s
    float move_base_rotation = 0.5f; // rad/s
};

#endif