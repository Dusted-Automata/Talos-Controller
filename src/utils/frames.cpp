#include "frames.hpp"
#include "cppmap3d.hh"
#include "transformations.hpp"
#include "types.hpp"
#include <iostream>

void
Frames::move_in_local_frame(Velocity2d velocity, const double dt)
{

    velocity.linear *= dt;
    velocity.angular *= dt;
    local_frame.orientation.rotate(Eigen::AngleAxisd((velocity.angular.z()), Vector3d::UnitZ()));
    local_frame.pos += local_frame.orientation.rotation() * velocity.linear;

    Ecef new_local_position = cppmap3d::enu2ecef(local_frame.pos, local_frame.origin);
    global_frame.orientation.rotate(Eigen::AngleAxisd(velocity.angular.z(), Vector3d::UnitY()));
    global_frame.pos = new_local_position;
}

void
Frames::move_in_global_frame(Velocity2d velocity, const double dt)
{
    velocity.linear *= dt;
    velocity.angular *= dt;

    local_frame.orientation.rotate(Eigen::AngleAxisd((velocity.angular.z()), Vector3d::UnitZ()));
    Linear_Velocity local_vel = local_frame.orientation.rotation() * velocity.linear;
    local_frame.pos += local_vel;
}

void
Frames::update_based_on_measurement(const LLH &llh)
{
    // std::cout << "LOCAL_FRAME: " << local_frame.pos.transpose() << std::endl;
    Ecef measured_ecef = cppmap3d::geodetic2ecef(llh);
    std::cout << std::fixed;
    std::cout << "ECEF: " << measured_ecef.raw().transpose() << std::endl;
    // std::cout << "GLOB : " << global_frame.pos.transpose() << std::endl;
    ENU enu = cppmap3d::ecef2enu(measured_ecef, local_frame.origin);
    local_frame.pos = enu;
}
void
Frames::init(Robot_Path &path)
{
    if (!path.current().has_value()) {
        return;
    }
    local_frame.origin = cppmap3d::ecef2geodetic(path.current()->point);
    global_frame.pos = path.current()->point;

    // Eigen::Matrix3d M = wgs_ecef2ned_matrix(llh);
    // global_frame.orientation = M;

    double theta = atan2(local_frame.origin.lon(), local_frame.origin.lat());
    Eigen::AngleAxisd rot_yaw(theta, Vector3d::UnitZ());
    local_frame.orientation = rot_yaw.toRotationMatrix();

    if (!path.next().has_value()) {
        return;
    }

    ENU goal = cppmap3d::ecef2enu(path.next()->point, local_frame.origin);
    double goal_theta = atan2(goal.east(), goal.north());
    Eigen::AngleAxisd rot_yaw_goal(goal_theta, Vector3d::UnitZ());
    local_frame.orientation = local_frame.orientation * rot_yaw_goal;
}
