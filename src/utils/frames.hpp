#pragma once
#include "robot_path.hpp"
#include "types.hpp"

struct Frame {
    // Eigen::Quaterniond orientation;
    Affine3d orientation = Affine3d::Identity();
    Vector3d pos{ 0, 0, 0 };
};

struct ENU_Frame : public Frame {
    LLH origin;
    ENU pos{ 0, 0, 0 };
};

struct Ecef_Frame : public Frame {
    Ecef pos{ 0, 0, 0 };
};

struct Frames {
    ENU_Frame local_frame;
    Ecef_Frame global_frame;
};

// class Frames
// {
//   public:
//     // Body frame to local_frame to global_frame
//     // Body frame being the Identity, since it is always on top of the robot
//     // local frame being NED
//     // global frame being ecef.
//     ENU_Frame local_frame;
//     Ecef_Frame global_frame;
//
//     void init(Robot_Path &path);
//     void move_in_local_frame(Velocity2d velocity, const double dt);
//     void move_in_global_frame(Velocity2d velocity, const double dt);
//
//     void update_based_on_measurement(const LLH &llh);
//     Vector3d get_error_vector_in_NED(const LLH &llh);
// };

void frames_init(Frames &frames, Robot_Path &path);
void frames_move_in_local_frame(Frames &frames, Velocity2d velocity, const double dt);
// void frames_move_in_global_frame(Velocity2d velocity, const double dt);

void frames_update_based_on_measurement(Frames &frames, const LLH &llh);

Vector3d frames_diff(const Frames &frames, const ENU &goal);

double frames_dist(const Vector3d &diff);
