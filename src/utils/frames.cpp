#include "frames.hpp"
#include "cppmap3d.hh"
#include "transformations.hpp"
#include "types.hpp"
#include <iostream>

void
Frames::move_in_local_frame(const Velocity2d &velocity)
{
    local_frame.orientation.rotate(Eigen::AngleAxisd((velocity.angular.z()), Vector3d::UnitZ()));
    local_frame.pos += local_frame.orientation.rotation() * velocity.linear;
    // std::cout << local_frame.pos.raw().transpose() << std::endl;
    // std::cout << local_frame.orientation.rotation() * velocity.linear << std::endl;

    Ecef new_local_position = cppmap3d::enu2ecef(local_frame.pos, local_frame.origin);
    // Angular_Velocity test = local_frame.orientation.rotation() * velocity.angular;
    global_frame.orientation.rotate(Eigen::AngleAxisd(velocity.angular.z(), Vector3d::UnitY()));
    // std::cout << new_local_position.raw().transpose() << std::endl;
    global_frame.pos = new_local_position;
}

void
Frames::move_in_global_frame(const Velocity2d &velocity)
{
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

// Vector3d
// Frames::get_error_vector_in_NED(const LLH &llh)
// {
//     Ecef measured_ecef = wgsllh2ecef(llh.lat(), llh.lon(), llh.alt());
//     Vector3d robot_ecef = cppmap3d::ecef2ned(local_frame.pos, local_frame.origin);
//     Ecef error_vec = robot_ecef - measured_ecef;
//     return wgsecef2ned(error_vec, local_frame.origin);
// }

// void
// Frames::init(const std::vector<Ecef> &waypoints)
// {
//     if (waypoints.size() < 1) {
//         return;
//     }
//     local_frame.origin = waypoints.front();
//     global_frame.pos = waypoints.front();
//
//     LLH llh = wgsecef2llh(local_frame.origin);
//     Eigen::Matrix3d M = wgs_ecef2ned_matrix(llh);
//     global_frame.orientation = M;
//
//     // Maybe necessary for correct local orientation at the start
//     // double theta = atan2(llh.y(), llh.x());
//     // Eigen::AngleAxisd rot_yaw(theta, Vector3d::UnitZ());
//     // local_frame.orientation = rot_yaw.toRotationMatrix();
//
//     if (waypoints.size() < 2) {
//         return;
//     }
//
//     Vector3d goal = wgsecef2ned_d(waypoints[1], local_frame.origin);
//     double theta = atan2(goal.y(), goal.x());
//     Eigen::AngleAxisd rot_yaw(theta, Vector3d::UnitZ());
//     local_frame.orientation = rot_yaw.toRotationMatrix();
// }

void
Frames::init(const std::vector<Pose> &waypoints)
{
    if (waypoints.size() < 1) {
        return;
    }
    local_frame.origin = cppmap3d::ecef2geodetic(waypoints.front().point);
    global_frame.pos = waypoints.front().point;

    // Eigen::Matrix3d M = wgs_ecef2ned_matrix(llh);
    // global_frame.orientation = M;

    double theta = atan2(local_frame.origin.lon(), local_frame.origin.lat());
    Eigen::AngleAxisd rot_yaw(theta, Vector3d::UnitZ());
    local_frame.orientation = rot_yaw.toRotationMatrix();

    if (waypoints.size() < 2) {
        return;
    }

    ENU goal = cppmap3d::ecef2enu(waypoints[1].point, local_frame.origin);
    double goal_theta = atan2(goal.east(), goal.north());
    Eigen::AngleAxisd rot_yaw_goal(goal_theta, Vector3d::UnitZ());
    local_frame.orientation = local_frame.orientation * rot_yaw_goal;
}
