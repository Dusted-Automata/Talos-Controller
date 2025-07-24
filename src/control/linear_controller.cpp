#include "linear_controller.hpp"
#include "cppmap3d.hh"
#include "frames.hpp"
#include "robot.hpp"
#include "transformations.hpp"
#include "types.hpp"
#include <cmath>

Velocity2d
Linear_Controller::calculate_cmd(Robot &robot, Motion_Profile &motion_profile, double dt)
{
    Velocity2d cmd = { .linear = Linear_Velocity().setZero(), .angular = Angular_Velocity().setZero() };

    std::optional<Pose> target_waypoint = robot.path.next();
    if (!target_waypoint.has_value()) {
        motion_profile.reset();
        return cmd;
    }
    ENU goal = cppmap3d::ecef2enu(target_waypoint.value().point, robot.frames.local_frame.origin);
    Vector3d diff = goal.raw() - robot.frames.local_frame.pos.raw();
    diff = robot.frames.local_frame.orientation.rotation().transpose() * diff;
    double dist = sqrt(diff.x() * diff.x() + diff.y() * diff.y());

    double yaw_error = atan2(diff.y(), diff.x());

    if (dist < robot.config.goal_tolerance_in_meters) {
        robot.path.progress();
        motion_profile.reset();

        std::optional<Pose> target_waypoint = robot.path.next();
        if (target_waypoint.has_value()) {
            ENU goal = cppmap3d::ecef2enu(target_waypoint.value().point, robot.frames.local_frame.origin);
            Vector3d diff = goal.raw() - robot.frames.local_frame.pos.raw();
            diff = robot.frames.local_frame.orientation.rotation().transpose() * diff;
            double dist = sqrt(diff.x() * diff.x() + diff.y() * diff.y());
            motion_profile.set_setpoint(dist);
        }
        return cmd;
    }

    // std::cout << dist << std::endl;
    motion_profile.update(dist - robot.config.goal_tolerance_in_meters, dt);

    cmd.linear.x() = motion_profile.velocity, robot.pose_state.velocity.linear.x();
    cmd.linear.y() = 0.0;
    cmd.linear.z() = 0.0;
    cmd.angular.z() = yaw_error;
    cmd.angular.y() = 0.0;
    cmd.angular.x() = 0.0;
    return cmd;
}

Velocity2d
Linear_Controller::get_cmd(Robot &robot, double dt)
{
    Velocity2d cmd = calculate_cmd(robot, motion_profile, dt);
    cmd.linear.x() = linear_pid.update(cmd.linear.x(), robot.pose_state.velocity.linear.x(), dt);
    cmd.angular.z() = angular_pid.update(0, -cmd.angular.z(), dt);
    return cmd;
}
