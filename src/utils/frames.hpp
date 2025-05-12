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

class Frames
{
  public:
    // Body frame to local_frame to global_frame
    // Body frame being the Identity, since it is always on top of the robot
    // local frame being NED
    // global frame being ecef.
    NED_Frame local_frame;
    Frame global_frame;

    void init(const std::optional<Ecef_Coord> &coordinate);
    void init(const std::optional<std::pair<Ecef_Coord, Ecef_Coord>> &path);
    void move_in_local_frame(const Velocity2d &velocity);

    void move_in_global_frame(const Velocity2d &velocity);

    void update_based_on_measurement(const double &lat, const double &lng, const double &height);
    Vector3d get_error_vector_in_NED(const double &lat, const double &lng, const double &height);
};
