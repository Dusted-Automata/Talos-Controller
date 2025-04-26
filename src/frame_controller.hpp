#pragma once
#include "types.hpp"

struct Frame {
    // Eigen::Quaterniond orientation;
    Affine3d orientation = Affine3d::Identity();
    Vector3d pos = Vector3d::Zero();
};

struct NED_Frame : public Frame {
    Ecef_Coord origin;
};

class Frame_Controller
{
  public:
    // Body frame to local_frame to global_frame
    // Body frame being the Identity, since it is always on top of the robot
    // local frame being NED
    // global frame being ecef.
    NED_Frame local_frame;
    Frame global_frame;

    void move_in_local_frame(Velocity2d &velocity);

    void move_in_global_frame(Velocity2d &velocity);
};
