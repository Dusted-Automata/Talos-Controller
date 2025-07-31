#include "linear_controller.hpp"
#include "frames.hpp"
#include "transformations.hpp"
#include "types.hpp"
#include <cmath>
#include <iostream>

Velocity2d
Linear_Controller::get_cmd(Pose_State pose_state, Vector3d diff, Vector3d motion_profile_diff, double dt)
{
    Velocity2d cmd = { .linear = Linear_Velocity().setZero(), .angular = Angular_Velocity().setZero() };

    double yaw_error = atan2(diff.y(), diff.x());
    std::cout << "yaw_error: " << yaw_error << std::endl;
    // motion_profile.update(frames_dist(diff) - goal_tolerance_in_meters, dt);
    motion_profile.update(frames_dist(motion_profile_diff) - goal_tolerance_in_meters, dt);

    cmd.linear.x() = motion_profile.velocity;
    cmd.linear.y() = 0.0;
    cmd.linear.z() = 0.0;
    cmd.angular.z() = yaw_error;
    cmd.angular.y() = 0.0;
    cmd.angular.x() = 0.0;

    // cmd.linear.x() = linear_pid.update(cmd.linear.x(), pose_state.velocity.linear.x(), dt);
    cmd.angular.z() = angular_pid.update(0, -cmd.angular.z(), dt);
    return cmd;
}
