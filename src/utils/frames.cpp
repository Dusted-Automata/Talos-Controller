#include "frames.hpp"
#include "cppmap3d.hh"
#include "transformations.hpp"
#include "types.hpp"
#include "math.hpp"
#include <iostream>

void
frames_move_in_local_frame(Frames &frames, Linear linear_movement, Angular angular_movement,  const double dt)
{

    auto position_update = linear_movement.velocity * dt;
    auto angle_update = angular_movement.velocity * dt;
    frames.local_frame.orientation.rotate(Eigen::AngleAxisd((angle_update.z()), Vector3d::UnitZ()));
    frames.local_frame.pos += frames.local_frame.orientation.rotation() * position_update;

    {
        // double new_local_orientation = convert_to_positive_radians(
        //     atan2(frames.local_frame.orientation.rotation()(1, 0), frames.local_frame.orientation.rotation()(0, 0)));
        // std::cout << "move_in_local_frame: " << new_local_orientation << std::endl;
    }

    Ecef new_local_position = cppmap3d::enu2ecef(frames.local_frame.pos, frames.local_frame.origin);
    // frames.global_frame.orientation.rotate(Eigen::AngleAxisd(angle_update.z(), Vector3d::UnitZ()));
    frames.global_frame.pos = new_local_position;
    frames.global_frame.orientation = frames.global_frame.set_rotation * frames.local_frame.orientation.rotation();
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
    frames.global_frame.pos = measured_ecef;
}
void
frames_init(Frames &frames, Pose current, Pose next)
{
    // if (!current) {
    //     return;
    // }

    LLH llh = cppmap3d::ecef2geodetic(current.point);
    frames.local_frame.origin = llh;
    frames.global_frame.pos = current.point;
    frames.global_frame.start_pos = current.point;

    const double sL = std::sin(llh.lon());
    const double cL = std::cos(llh.lon());
    const double sB = std::sin(llh.lat());
    const double cB = std::cos(llh.lat());

    // ENU -> ECEF
    Eigen::Matrix3d R;
    R << -sL,       -cL*sB,    cL*cB,
          cL,       -sL*sB,    sL*cB,
          0,            cB,       sB;

    frames.global_frame.set_rotation = R;

    // double theta = atan2(frames.local_frame.origin.lon(), frames.local_frame.origin.lat());
    // Eigen::AngleAxisd rot_yaw(theta, Vector3d::UnitZ());
    // frames.local_frame.orientation = rot_yaw.toRotationMatrix();

    // if (!next) {
    //     return;
    // }

    ENU goal = next.local_point;
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

