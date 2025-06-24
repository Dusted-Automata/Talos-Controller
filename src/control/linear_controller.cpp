#include "linear_controller.hpp"
#include "cppmap3d.hh"
#include "frames.hpp"
#include "robot.hpp"
#include "transformations.hpp"
#include "types.hpp"
#include <cmath>

bool
above_epsilon(double lat, double lng, double alt)
{
    if (std::abs(lat) > 0.001 || std::abs(lng) > 0.001 || std::abs(alt) > 0.001) {
        return true;
    }
    return false;
}

Velocity2d
Linear_Controller::calculate_cmd(Robot &robot, Motion_Profile &motion_profile, double dt)
{
    Velocity2d cmd = { .linear = Linear_Velocity().setZero(), .angular = Angular_Velocity().setZero() };
    // auto ublox_gga = robot->ublox.get_latest<GGA>(Msg_Type::NAV_ATT);
    // if (ublox_gga.has_value()) {
    //     GGA val = ublox_gga.value().val;
    //     double lat = to_radian(val.latlng.lat);
    //     double lng = to_radian(val.latlng.lng);
    //     double alt = val.alt;
    //
    //     if (above_epsilon(lat, lng, alt)) {
    //         // Vector3d error_vec = robot->frames.get_error_vector_in_NED(lat, lng, alt);
    //         robot->frames.update_based_on_measurement({ lat, lng, alt });
    //     }
    //     robot->ublox.consume_measurement(Msg_Type::NAV_ATT);
    // }

    auto ublox_simple = robot.ublox.get_latest<Nav_Att>(Msg_Type::NAV_ATT);
    if (ublox_simple.has_value()) {
        Nav_Att nav_att = ublox_simple.value();
        double heading = to_radian(nav_att.heading);
        std::cout << nav_att.heading << std::endl;
        // std::cout << heading << std::endl;
        Eigen::AngleAxisd yawAngle(heading, Eigen::Vector3d::UnitZ());
        Eigen::Matrix3d rotationMatrix = yawAngle.toRotationMatrix();
        robot.frames.local_frame.orientation.linear() + rotationMatrix;
    }

    std::optional<Pose> target_waypoint = robot.path.get_next();
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
        robot.path.pop();
        motion_profile.reset();

        std::optional<Pose> target_waypoint = robot.path.get_next();
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
