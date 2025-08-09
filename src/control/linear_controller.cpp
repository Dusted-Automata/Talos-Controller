#include "linear_controller.hpp"
#include "frames.hpp"
#include "transformations.hpp"
#include "types.hpp"
#include "ublox.hpp"
#include <cmath>
#include <iostream>

Velocity2d
Linear_Controller::get_cmd(Pose_State pose_state, Vector3d diff, Vector3d motion_profile_diff, double dt)
{
    Velocity2d cmd = { .linear_vel = Linear_Velocity().setZero(), .angular_vel = Angular_Velocity().setZero() };

    double yaw_error = atan2(diff.y(), diff.x());
    if (convert_to_positive_radians(yaw_error) < to_radian(10)) {
        aligned_to_goal_waypoint = true;
    }
    // double yaw_error = atan2(diff.y(), diff.x());
    // std::cout << "yaw_error: " << yaw_error << std::endl;
    // motion_profile.update(frames_dist(diff) - goal_tolerance_in_meters, dt);
    motion_profile.update(eucledean_xy_norm(motion_profile_diff) - goal_tolerance_in_meters, dt);

    if (aligned_to_goal_waypoint) {
        cmd.linear_vel.x() = motion_profile.velocity;
    }
    // cmd.linear_vel.x() = motion_profile.velocity;
    cmd.linear_vel.y() = 0.0;
    cmd.linear_vel.z() = 0.0;
    // cmd.angular_vel.z() = yaw_error;
    cmd.angular_vel.y() = 0.0;
    cmd.angular_vel.x() = 0.0;

    double yaw_error_offset = to_radian(-50);
    // cmd.linear.x() = linear_pid.update(cmd.linear.x(), pose_state.velocity.linear.x(), dt);
    // cmd.angular_vel.z() = angular_pid.update(0, -(yaw_error + yaw_error_offset), dt);
    // cmd.angular_vel.z() = angular_pid.update(yaw_error_offset, -yaw_error, dt);
    // cmd.angular_vel.z() = angular_pid.update(yaw_error_offset, -yaw_error, dt);
    cmd.angular_vel.z() = angular_pid.update(0, -yaw_error, dt);

    return cmd;
}
