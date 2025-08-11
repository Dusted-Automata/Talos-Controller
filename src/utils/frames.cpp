#include "frames.hpp"
#include "cppmap3d.hh"
#include "transformations.hpp"
#include "types.hpp"
#include "ublox.hpp"
#include <iostream>

void
frames_move_in_local_frame(Frames &frames, Velocity2d velocity, const double dt)
{

    auto position_update = velocity.linear_vel * dt;
    auto angle_update = velocity.angular_vel * dt;
    frames.local_frame.orientation.rotate(Eigen::AngleAxisd((angle_update.z()), Vector3d::UnitZ()));
    frames.local_frame.pos += frames.local_frame.orientation.rotation() * position_update;

    {
        double new_local_orientation = convert_to_positive_radians(
            atan2(frames.local_frame.orientation.rotation()(1, 0), frames.local_frame.orientation.rotation()(0, 0)));
        // std::cout << "move_in_local_frame: " << new_local_orientation << std::endl;
    }

    // Ecef new_local_position = cppmap3d::enu2ecef(frames.local_frame.pos, frames.local_frame.origin);
    // frames.global_frame.orientation.rotate(Eigen::AngleAxisd(angle_update.z(), Vector3d::UnitZ()));
    // frames.global_frame.pos = new_local_position;
}

void
frames_update_based_on_measurement(Frames &frames, const LLH &llh)
{
    // std::cout << "LOCAL_FRAME: " << local_frame.pos.transpose() << std::endl;
    // Ecef measured_ecef;
    // cppmap3d::geodetic2ecef(to_radian(llh.lat()), to_radian(llh.lon()), llh.alt(), measured_ecef.x(),
    // measured_ecef.y(),
    //     measured_ecef.z());
    //
    Ecef measured_ecef = cppmap3d::geodetic2ecef(llh);
    std::cout << std::fixed;
    // std::cout << "ECEF: " << measured_ecef.raw().transpose() << std::endl;
    // std::cout << "GLOB : " << global_frame.pos.transpose() << std::endl;
    ENU enu = cppmap3d::ecef2enu(measured_ecef, frames.local_frame.origin);
    frames.local_frame.pos = enu;
}
void
frames_init(Frames &frames, Robot_Path &path)
{
    if (path.size() < 1) {
        return;
    }

    frames.local_frame.origin = cppmap3d::ecef2geodetic(path.current().point);
    frames.global_frame.pos = path.current().point;

    // Eigen::Matrix3d M = wgs_ecef2ned_matrix(llh);
    // frames.global_frame.orientation = M;

    // double theta = atan2(frames.local_frame.origin.lon(), frames.local_frame.origin.lat());
    // Eigen::AngleAxisd rot_yaw(theta, Vector3d::UnitZ());
    // frames.local_frame.orientation = rot_yaw.toRotationMatrix();

    if (path.size() < 2) {
        return;
    }

    ENU goal = path.next().local_point;
    double goal_theta = convert_to_positive_radians(atan2(goal.north(), goal.east()));
    // double goal_theta = convert_to_positive_radians(3.45575);
    Eigen::AngleAxisd rot_yaw_goal(goal_theta, Vector3d::UnitZ());
    frames.local_frame.orientation = frames.local_frame.orientation.rotation() * rot_yaw_goal;
}

Vector3d
frames_diff(const Frames &frames, const ENU &goal) // fine with running every frame
{

    Vector3d position_diff = goal.raw() - frames.local_frame.pos.raw();
    position_diff = frames.local_frame.orientation.rotation().transpose() * position_diff;
    return position_diff;
}

double
eucledean_xy_norm(const Vector3d &diff) // fine with running every frame
{
    return sqrt(diff.x() * diff.x() + diff.y() * diff.y());
}
