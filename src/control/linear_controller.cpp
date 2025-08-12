#include "linear_controller.hpp"
#include "frames.hpp"
#include "transformations.hpp"
#include "types.hpp"
#include "ublox.hpp"
#include <cmath>

Velocity2d
Linear_Controller::get_cmd(Pose_State pose_state, Vector3d diff, Vector3d motion_profile_diff, double dt)
{
    Velocity2d cmd = { .linear_vel = Linear_Velocity().setZero(), .angular_vel = Angular_Velocity().setZero() };

    double yaw_error = atan2(diff.y(), diff.x());
    if (fabs(yaw_error) < to_radian(1)) {
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

    // double yaw_error_val = 140;
    // double yaw_error_offset = std::signbit(yaw_error) ? to_radian(-yaw_error_val) : to_radian(yaw_error_val);
    // cmd.angular_vel.z() = angular_pid.update(yaw_error_offset, -yaw_error, dt);
    cmd.angular_vel.z() = angular_pid.update(0, -yaw_error, dt);

    return cmd;
}
