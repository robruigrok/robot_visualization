#include "robot_arm.hpp"

#include <cmath>
#include <iostream>

// Constructor

RoboticArm::RoboticArm() {
    // Hardcode robot: stack links in the way they are connected.
    // Entities in links: translation x, y z, rotation x, y, z, type, range_min, range_max, max_speed, max_acceleration
    float velocity_gain = 2.0f;
    float acceleration_gain = 2.0f;
    float range_gain = 4.0f;
    links = {
        RobotLink("move_base_xyz", 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    RobotLink::LinkType::Move_Base, 0.0, 0.0f, 0.0f, 0.0f),
        RobotLink("move_base_rotz", 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                    RobotLink::LinkType::Move_Base, 0.0, 0.0f, 0.0f, 0.0f),      
        // Base: 1.5m tall, 0.3m x 0.3m
        RobotLink("base", 0.0f, 0.0f, 1.5f, 0.0f, 0.0f, 0.0f,
                    RobotLink::LinkType::Z, 1.0f, 2.0f, velocity_gain*0.2f, acceleration_gain*0.2f),
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

// Similate the links and base one step

void RoboticArm::update() {
    float dt = getUpdateInterval() / 1000.0f; // Convert ms to seconds
    if (!move_mode_set_joints){
        moveBase(); // move the base
    }
        
    for (auto &link : links)
    {
        // call position controller of the link
        float acc = link.computePositionControl(link.getRequestedPosition(), dt);
        // simulate the link
        link.simulate(acc, dt);
    }
}

void RoboticArm::moveBase() {
    // Get current joint goal angles, required for computing feed forward
    auto [g1, g2, g3] = getLinkGoalPositions("arm1", "arm2", "arm3");
    
    // compute the velocity of the base. This updates the velocity in the move_base object.
    computeMoveBaseVelocity();
    
    // move the base one step using the originally planned velocity
    utils::Pose new_pose_candidate = move_base.pose;
    new_pose_candidate.x += move_base.velocity.vel_x * getUpdateInterval() / 1000.0f;
    new_pose_candidate.y += move_base.velocity.vel_y * getUpdateInterval() / 1000.0f;
    new_pose_candidate.z += move_base.velocity.vel_z * getUpdateInterval() / 1000.0f;
    new_pose_candidate.rot_z += move_base.velocity.rot_z * getUpdateInterval() / 1000.0f;
   
    // express the goal position in the candidate base pose
    utils::Pose goal_pose_wrt_robot = convertGoalPoseInBaseFrame(goal_pose_world, new_pose_candidate);

    // Check whether the joint can keep up with proposed base velocity.
    float move_base_scaling_ratio = 1.0f;
    try
    {
        // get the required joint angles for the goal pose in candidate frame.
        auto [a1, a2, a3] = computeJointAngles(goal_pose_wrt_robot.x, goal_pose_wrt_robot.y, goal_pose_wrt_robot.rot_z);
        
        // compute required velocity for each joint
        float ff_1 = (a1 - g1) / (getUpdateInterval() / 1000.0f); // TODO: check for wrapping around
        float ff_2 = (a2 - g2) / (getUpdateInterval() / 1000.0f);
        float ff_3 = (a3 - g3) / (getUpdateInterval() / 1000.0f);

        // get achievable velocity for each link
        float scaling_ratio = ratioUsedByFeedForward("arm1", ff_1, "arm2", ff_2, "arm3", ff_3, "base", -move_base.velocity.vel_z);
        
        // If the base motion requires more than 80% of the max speed of some joint, reduce the velocities
        move_base_scaling_ratio = std::min(0.80f/scaling_ratio, 1.0f);

        // output the scaling ratio:
        std::cout << "Scaling used for slowing base down: " << move_base_scaling_ratio << std::endl;

        // Come up with a new pose candidate for the base, using the scaling ratio
        new_pose_candidate = move_base.pose;
        new_pose_candidate.x += move_base_scaling_ratio*move_base.velocity.vel_x * getUpdateInterval() / 1000.0f;
        new_pose_candidate.y += move_base_scaling_ratio*move_base.velocity.vel_y * getUpdateInterval() / 1000.0f;
        new_pose_candidate.z += move_base_scaling_ratio*move_base.velocity.vel_z * getUpdateInterval() / 1000.0f;
        new_pose_candidate.rot_z += move_base_scaling_ratio*move_base.velocity.rot_z * getUpdateInterval() / 1000.0f;

        // with this new pose, compute the goal pose in the base frame again
        utils::Pose goal_pose_wrt_robot = convertGoalPoseInBaseFrame(goal_pose_world, new_pose_candidate);

        // compute the joint angles again for the final base frame.
        auto [b1, b2, b3] = computeJointAngles(goal_pose_wrt_robot.x, goal_pose_wrt_robot.y, goal_pose_wrt_robot.rot_z);

        // Set the resulting joint angles as requested angles for position control
        setRequestedAngles(b1, b2, b3);

        std::cout << "Computed joint angles: " << b1 << ", " << b2 << ", " << b3 << std::endl;

        // I think we could in theory just scale the computed FF velocities, but I compute them again anyway.
        // Just to be sure, compute the FF velocity again
        ff_1 = (b1 - g1) / (getUpdateInterval() / 1000.0f); // TODO: check for wrapping around
        ff_2 = (b2 - g2) / (getUpdateInterval() / 1000.0f);
        ff_3 = (b3 - g3) / (getUpdateInterval() / 1000.0f);

        // for debugging, print the ff values
        std::cout << "Feed forward velocities scaled: ff_1=" << ff_1
                  << ", ff_2=" << ff_2
                  << ", ff_3=" << ff_3
                  << ", move_base_vel_z=" << move_base_scaling_ratio*-move_base.velocity.vel_z << std::endl;

        // set FF velocity for each joint
        setFeedForwardVelocity("arm1", ff_1, "arm2", ff_2, "arm3", ff_3, "base", -move_base_scaling_ratio*move_base.velocity.vel_z);

    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    // With the new pose_candidate found, update the base
    move_base.pose = new_pose_candidate; // update the pose of the base
    // Set the velocity of the base to the recuded velocity
    move_base.velocity.vel_x = move_base_scaling_ratio * move_base.velocity.vel_x;
    move_base.velocity.vel_y = move_base_scaling_ratio * move_base.velocity.vel_y;
    move_base.velocity.vel_z = move_base_scaling_ratio * move_base.velocity.vel_z;
    move_base.velocity.rot_z = move_base_scaling_ratio * move_base.velocity.rot_z;

    // update this pose in the respective links
    for (auto &link : links)
    {
        if (link.getLinkName() == "move_base_xyz")
        {   
            // update the pose of the link for the frontend
            utils::Pose new_pose_link = link.getPose();
            new_pose_link.x = new_pose_candidate.x;
            new_pose_link.y = new_pose_candidate.y;
            new_pose_link.z = new_pose_candidate.z;
            link.setPose(new_pose_link);
        } else if (link.getLinkName() == "move_base_rotz")
        {
            // update the pose of the link for the frontend
            utils::Pose new_pose_link = link.getPose();
            new_pose_link.rot_z = new_pose_candidate.rot_z; // only update rotation
            link.setPose(new_pose_link);
        }
    }

    // Set the goal position for position control
    setGoalPose(goal_pose_wrt_robot.x, goal_pose_wrt_robot.y, goal_pose_wrt_robot.z, goal_pose_wrt_robot.rot_z);

    // Set goal positions for link in z position.
    computeZMotion();
    
}

//Getters

std::tuple<float, float, float> RoboticArm::getLinkLengths(const std::string& arm1, const std::string& arm2, const std::string& arm3) const {
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

std::tuple<float, float, float> RoboticArm::getCurrentAngles(const std::string& arm1, const std::string& arm2, const std::string& arm3) const {
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

std::tuple<float, float, float> RoboticArm::getLinkGoalPositions(const std::string& arm1, const std::string& arm2, const std::string& arm3) const {
float rot1 = 0.0f, rot2 = 0.0f, rot3 = 0.0f;
    for ( auto& link : links) {
        const std::string& name = link.getLinkName();
        if (name == arm1) {
            rot1 = link.getRequestedPosition();
        } else if (name == arm2) {
            rot2 = link.getRequestedPosition();
        } else if (name == arm3) {
            rot3 = link.getRequestedPosition();
        }
    }
    return {rot1, rot2, rot3};
}

json RoboticArm::getState() const {
    json state = json::array();
    for (const auto &link : links)
    {
        state.push_back(link.getState());
    }
    return state;
}

std::vector<RobotLink>& RoboticArm::getLinks() {
    return links;
}

int RoboticArm::getUpdateInterval() const { 
    return update_interval_ms;
}

// Setters

void RoboticArm::setGoalPose(float x, float y, float z, float rotz) {
    goal_x = x;
    goal_y = y;
    goal_z = z;
    goal_rot_z = rotz; // should already be in radians
}

void RoboticArm::setGoalPoseWorld(float x, float y, float z, float rotz) {
    goal_pose_world.x = x;
    goal_pose_world.y = y;
    goal_pose_world.z = z;
    goal_pose_world.rot_z = rotz; // should already be in radians
}

void RoboticArm::setMoveBaseGoalPose(float x, float y, float z, float rotz) {
    move_base_goal.x = x;
    move_base_goal.y = y;
    move_base_goal.z = z;
    move_base_goal.rot_z = rotz; // should already be in radians
}

void RoboticArm::setFeedForwardVelocity(const std::string& arm1, float ff1, const std::string& arm2, float ff2, const std::string& arm3, float ff3, const std::string& base, float ffz) {
    for ( auto& link : links) {
        const std::string& name = link.getLinkName();
        if (name == arm1) {
            link.setFeedForwardVelocity(ff1);
        } else if (name == arm2) {
            link.setFeedForwardVelocity(ff2);
        } else if (name == arm3) {
            link.setFeedForwardVelocity(ff3);
        } else if (name == base) {
            link.setFeedForwardVelocity(ffz);
        }
    }
}

void RoboticArm::setRequestedAngles(float rot1, float rot2, float rot3) {
    for (auto& link : links) {
        const std::string& name = link.getLinkName();
        if (name == "arm1") {
            link.setRequestedPosition(rot1);
        } else if (name == "arm2") {
            link.setRequestedPosition(rot2);
        } else if (name == "arm3") {
            link.setRequestedPosition(rot3);
        }
    }
}

void RoboticArm::setMoveModeSetJoints(bool mode) {
    move_mode_set_joints = mode;
}

// Conversions and computations

utils::Pose RoboticArm::convertGoalPoseInBaseFrame(utils::Pose goal_pose, utils::Pose move_base) {
    // Get base pose in world frame
    float x_r = move_base.x;
    float y_r = move_base.y;
    float z_r = move_base.z;
    float rot_z_r = move_base.rot_z;

    // translat the goal pose
    float x_trans = goal_pose.x - x_r;
    float y_trans = goal_pose.y - y_r;
    float z_trans = goal_pose.z - z_r;

    // rotate base
    float cos_theta = cos(-rot_z_r);
    float sin_theta = sin(-rot_z_r);
    
    utils::Pose goal_pose_wrt_robot;
    goal_pose_wrt_robot.x = x_trans * cos_theta - y_trans * sin_theta;
    goal_pose_wrt_robot.y = x_trans * sin_theta + y_trans * cos_theta;
    goal_pose_wrt_robot.z = z_trans;
    goal_pose_wrt_robot.rot_z = goal_pose.rot_z - rot_z_r; // Relative orientation    

    return goal_pose_wrt_robot;
}

std::tuple<float, float, float>  RoboticArm::computeJointAngles(float x, float y, float rot_z) {
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
        // create exception or return a default value
        throw std::runtime_error("Target position is unreachable");

        return {0,0,0};
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
        float distance = std::pow(utils::getAngleDifference (solutions[i].rot1, current_rot1), 2) +
                        std::pow(utils::getAngleDifference (solutions[i].rot2, current_rot2), 2) +
                        std::pow(utils::getAngleDifference (solutions[i].rot3, current_rot3), 2);
        if (distance < min_distance) {
            min_distance = distance;
            best_solution = i;
        }
    }

    return {solutions[best_solution].rot1, solutions[best_solution].rot2, solutions[best_solution].rot3};
    // // return solutions[best_solution];
    // setRequestedAngles(solutions[best_solution].rot1, solutions[best_solution].rot2, solutions[best_solution].rot3);
    // std::cout << "Computed joint angles: " << solutions[best_solution].rot1 << ", "
    //             << solutions[best_solution].rot2 << ", " << solutions[best_solution].rot3 << std::endl;
    
}

float RoboticArm::ratioUsedByFeedForward(const std::string& arm1, float ff1, const std::string& arm2, float ff2, const std::string& arm3, float ff3, const std::string& base, float ffz){
    float r1 = 1.0f, r2 = 1.0f, r3 = 1.0f, r4 = 1.0f;
    for (auto& link : links) {
        const std::string& name = link.getLinkName();
        if (name == arm1) {
            r1 = std::abs(ff1/link.getMaxVelocity());
        } else if (name == arm2) {
            r2 = std::abs(ff2/link.getMaxVelocity());
        } else if (name == arm3) {
            r3 = std::abs(ff3/link.getMaxVelocity());
        } else if (name == base) {
            r4 = std::abs(ffz/link.getMaxVelocity());
        }
    }
    return std::max({r1, r2, r3, r4}); // return the maximum ratio
}

void RoboticArm::computeZMotion() {
    // loop over al links, get height of current tool
    float tool_height = 0.0f;
    for (auto& link : links) {
        if (link.getLinkName() == "move_base_xyz") {
            // skip the offset
            continue;
        }
        tool_height += link.getTranslationZ();
    }

    float z_error = goal_z - tool_height;

    // simply add this value to the base link
    for (auto& link : links) {
        if (link.getLinkName() == "base") {
            link.setRequestedPosition(link.getCurrentPosition() + z_error);
            break;
        }
    }
}

void RoboticArm::computeMoveBaseVelocity()
{        
    // Step 1: Compute translation differences
    float dx = move_base_goal.x - move_base.pose.x;
    float dy = move_base_goal.y - move_base.pose.y;
    float dz = move_base_goal.z - move_base.pose.z;

    // Step 2: Compute shortest angular difference (handles wrapping)
    float angular_error = move_base_goal.rot_z - move_base.pose.rot_z;
    angular_error = fmod(angular_error + M_PI, 2 * M_PI) - M_PI; 

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
    } else {
        move_base.velocity.vel_x = 0.0f;
        move_base.velocity.vel_y = 0.0f;
        move_base.velocity.vel_z = 0.0f;
    }

// Step 7: Compute angular velocity
    float angular_time_to_complete = fabs(angular_error) / move_base_rotation; // Time to complete rotation at max speed
    if (angular_time_to_complete < update_interval_ms/1000.0f) {
        // If angular motion would complete within one update cycle, scale velocity
        move_base.velocity.rot_z = angular_error / (update_interval_ms/1000.0f);
    } else if (fabs(angular_error) > 1e-6) {
        // Synchronize with linear motion if possible, else use max rotation speed
        if (time_to_complete > 1e-6) {
            move_base.velocity.rot_z = angular_error / time_to_complete;
        } else {
            // If no linear motion, rotate at max speed
            move_base.velocity.rot_z = angular_error > 0 ? move_base_rotation : -move_base_rotation;
        }
    } else {
        move_base.velocity.rot_z = 0.0f;
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
