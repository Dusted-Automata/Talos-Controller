#pragma once
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

class Frames
{
  public:
    // Body frame to local_frame to global_frame
    // Body frame being the Identity, since it is always on top of the robot
    // local frame being NED
    // global frame being ecef.
    ENU_Frame local_frame;
    Ecef_Frame global_frame;

    void init(const std::optional<Ecef> &coordinate);
    void init(const std::vector<Ecef> &waypoints);
    void move_in_local_frame(const Velocity2d &velocity);
    void move_in_global_frame(const Velocity2d &velocity);

    void update_based_on_measurement(const LLH &llh);
    Vector3d get_error_vector_in_NED(const LLH &llh);
};
