#include "frame_controller.hpp"
#include "transformations.hpp"
#include "types.hpp"
#include <iostream>

void
Frame_Controller::move_in_local_frame(Velocity2d &velocity)
{
    local_frame.orientation.rotate(Eigen::AngleAxisd((velocity.angular.z()), Vector3d::UnitZ()));
    local_frame.pos += local_frame.orientation.rotation() * velocity.linear;
    // std::cout << local_frame.orientation.rotation() * velocity.linear << std::endl;

    Vector3d linear_vector = wgsned2ecef(local_frame.orientation.rotation() * velocity.linear, local_frame.origin);
    Angular_Velocity test = local_frame.orientation.rotation() * velocity.angular;
    global_frame.orientation.rotate(Eigen::AngleAxisd(velocity.angular.z(), Vector3d::UnitY()));
    global_frame.pos += linear_vector;
}

void
Frame_Controller::move_in_global_frame(Velocity2d &velocity)
{
    local_frame.orientation.rotate(Eigen::AngleAxisd((velocity.angular.z()), Vector3d::UnitZ()));
    Linear_Velocity local_vel = local_frame.orientation.rotation() * velocity.linear;
    local_frame.pos += local_vel;
}

void
Frame_Controller::update_based_on_measurement(double &lat, double &lng, double &height)
{
    // std::cout << "LOCAL_FRAME: " << local_frame.pos.transpose() << std::endl;
    Ecef_Coord measured_ecef = wgsllh2ecef(lat, lng, height);
    std::cout << std::fixed;
    std::cout << "ECEF: " << measured_ecef.transpose() << std::endl;
    std::cout << "GLOB : " << global_frame.pos.transpose() << std::endl;
    Vector3d ned = wgsecef2ned_d(measured_ecef, local_frame.origin);
    local_frame.pos = ned;
}

Vector3d
Frame_Controller::get_error_vector_in_NED(double &lat, double &lng, double &height)
{
    Ecef_Coord measured_ecef = wgsllh2ecef(lat, lng, height);
    Ecef_Coord robot_ecef = wgsned2ecef_d(local_frame.pos, local_frame.origin);
    Vector3d error_vec = robot_ecef - measured_ecef;
    return wgsecef2ned(error_vec, local_frame.origin);
}

void
Frame_Controller::init(Ecef_Coord &coordinate)
{
    local_frame.origin = coordinate;
    global_frame.pos = coordinate;

    LLH llh = wgsecef2llh(local_frame.origin);
    std::cout << "lat: " << llh[0] << " lng: " << llh[1] << " alt: " << llh[2] << std::endl;
    Eigen::Matrix3d M = wgs_ecef2ned_matrix(llh);
    double theta = atan2(llh.y(), llh.x());
    Eigen::AngleAxisd rot_yaw(theta, Vector3d::UnitZ());
    local_frame.orientation = rot_yaw.toRotationMatrix();
    global_frame.orientation = M;
}
