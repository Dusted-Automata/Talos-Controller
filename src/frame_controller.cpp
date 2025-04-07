#include "frame_controller.hpp"
#include "transformations.hpp"
#include "types.hpp"
#include <iostream>

void Frame_Controller::move_in_local_frame(Velocity2d &velocity)
{
    local_frame.orientation.rotate(Eigen::AngleAxisd((velocity.angular.z()), Vector3d::UnitZ()));
    local_frame.pos += local_frame.orientation.rotation() * velocity.linear;

    Vector3d linear_vector =
        wgsned2ecef(local_frame.orientation.rotation() * velocity.linear, local_frame.origin);
    global_frame.orientation.rotate(Eigen::AngleAxisd((velocity.angular.z()), Vector3d::UnitZ()));
    global_frame.pos += linear_vector;
}

void Frame_Controller::move_in_global_frame(Velocity2d &velocity)
{
    local_frame.orientation.rotate(Eigen::AngleAxisd((velocity.angular.z()), Vector3d::UnitZ()));
    Linear_Velocity local_vel = local_frame.orientation.rotation() * velocity.linear;
    local_frame.pos += local_vel;
}
