#pragma once
#include <Eigen/Dense>
#include <array>

using Eigen::Affine3d;
using Eigen::Matrix4d;
using Eigen::Vector3d;
typedef Vector3d Ecef_Coord;
typedef Vector3d LLH;
typedef Vector3d Linear_Velocity;
typedef Vector3d Angular_Velocity;

struct Robot_State
{
    std::array<float, 3> position; // (unit: m), from own odometry in inertial frame, usually drift
    std::array<float, 3> velocity; // (unit: m/s), forwardSpeed, sideSpeed,
                                   // rotateSpeed in body frame
    float yawSpeed;                // (unit: rad/s), rotateSpeed in body frame
                                   // std::array<MotorState, 20> motorState;
                                   // IMU imu;
};

// Similar to Trajectory_Point

// struct Angular_Velocity {
//   double roll, pitch, yaw;
// };
// struct Linear_Velocity {
//   double forward, lateral, vertical;
// };
struct Velocity2d
{
    // high-level representation
    // can have 12 joint angles/torques (3 per leg)
    Linear_Velocity linear;   // x, y, z velocity (m/s)
    Angular_Velocity angular; // roll, pitch, yaw rates (rad/s)
};

struct Pose_State
{
    Ecef_Coord position; // x, y, z
    Affine3d orientation;
    // Eigen::Quaterniond orientation; // quaternion
    Velocity2d velocity; // vx, vy, vz \  wx, wy, wz
    double dt;
};

struct Pose
{
    Ecef_Coord point;
    Affine3d transformation_matrix; // Change this to Quaternion maybe
};

struct Motion_Constraints
{
    double max_velocity;
    double min_velocity;
    double standing_turn_velocity;
    double max_acceleration;
    double max_deceleration;
    double max_jerk;
    double corner_velocity;
};

struct Velocity_Profile
{
    double time_to_max_speed;
    double time_to_min_speed;
    double corner_velocity;
    double standing_turn_velocity;
    double acceleration_rate;
    double deceleration_rate;
};

struct Robot_Config
{
    int hz;
    Motion_Constraints motion_constraints;
    Velocity_Profile velocity_profile;
    // TODO: Make a real frame and transformation data structure
    // Matrix4d robot_frame{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0,
    // 1}}; Matrix4d world_frame{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0,
    // 0, 1}};
    Affine3d transform_world_to_robot = Affine3d::Identity();
};

struct Motion_Step
{
    Ecef_Coord &current;
    Ecef_Coord &next;
    Ecef_Coord &difference;
    Affine3d &robot_frame;
    double &dt;
};

struct Trajectory_Point
{
    Pose pose;
    double dt;
    Velocity2d velocity;
};
