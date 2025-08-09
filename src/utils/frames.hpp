#pragma once
#include "robot_path.hpp"
#include "types.hpp"

struct Frame {
    // Eigen::Quaterniond orientation;
    Affine3d orientation = Affine3d::Identity();
    // Eigen::Matrix3d orientation = Eigen::Matrix3d::Identity();
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

void frames_init(Frames &frames, Robot_Path &path);
void frames_move_in_local_frame(Frames &frames, Velocity2d velocity, const double dt);
// void frames_move_in_global_frame(Velocity2d velocity, const double dt);

void frames_update_based_on_measurement(Frames &frames, const LLH &llh);

Vector3d frames_diff(const Frames &frames, const ENU &goal);

double eucledean_xy_norm(const Vector3d &diff);
