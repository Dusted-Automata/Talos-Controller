#include "linear_controller.hpp"
#include "frame_controller.hpp"
#include "transformations.hpp"
#include "robot.hpp"
#include "types.hpp"
#include <cmath>
#include <iostream>

void Linear_Controller::path_loop(std::vector<Ecef_Coord> &waypoints)
{
    if (waypoints.empty())
    {
        return;
    }
    // std::cout << path.size() << "  WAYPOINTS" << std::endl;
    if (!added_paths)
    {
        std::cout << "EMPTY FIRST WAYPOINTS" << std::endl;
        for (Ecef_Coord &waypoint : waypoints)
        {
            std::cout << "Adding Waypoints!" << std::endl;
            std::cout << waypoint.transpose() << std::endl;
            robot->path_queue.push(waypoint);
        }
        added_paths = true;
    }

    // FIXME: One waypoint does not get popped off, so it wont loop
    if ((robot->path_queue.size() == 1) && path_looping)
    {
        std::cout << robot->path_queue.size() << " EMPTY WAYPOINTS" << std::endl;
        for (Ecef_Coord &waypoint : waypoints)
        {
            std::cout << "Adding Waypoints!" << std::endl;
            std::cout << waypoint.transpose() << std::endl;
            robot->path_queue.push(waypoint);
        }
    }
}

Velocity2d Linear_Controller::get_cmd(Frame_Controller &frame_controller,
                                      Thread_Safe_Queue<Ecef_Coord> &path_queue)
{
    double max_vel_x = 2.0;
    double min_vel_x = -2.0;
    double goal_yaw = 0;
    double rotate_dist_threshold = 0.1;
    int odom_waiting_count = 1;

    double yaw_tolerance = 20.0; // degrees
    double goal_tolerance = 0.3; // meters

    double proportional_gain_x = 0.8;
    double proportional_gain_yaw = 1.0;

    Velocity2d cmd = {.linear = Linear_Velocity().setZero(),
                      .angular = Angular_Velocity().setZero()};

    std::optional<std::pair<Ecef_Coord, Ecef_Coord>> path = path_queue.front_two();
    if (!path.has_value())
    {
        return cmd;
        // linear_pid.reset();
        // angular_pid.reset();
        // path_queue.pop();
    }
    Ecef_Coord start = wgsecef2ned_d(path.value().first, frame_controller.local_frame.origin);
    Ecef_Coord goal = wgsecef2ned_d(path.value().second, frame_controller.local_frame.origin);

    Ecef_Coord difference = goal - start;
    double difference_distance =
        std::sqrt(difference.x() * difference.x() + difference.y() * difference.y());

    double dx = goal.x() - frame_controller.local_frame.pos.x();
    double dy = goal.y() - frame_controller.local_frame.pos.y();
    double dz = goal.z() - frame_controller.local_frame.pos.z();
    double dist = sqrt(dx * dx + dy * dy);

    double yaw = atan2(frame_controller.local_frame.orientation.rotation()(1, 0),
                       frame_controller.local_frame.orientation.rotation()(0, 0));

    double dx_odom = cos(yaw) * dx + sin(yaw) * dy;
    double dy_odom = -sin(yaw) * dx + cos(yaw) * dy;

    double vel_x = proportional_gain_x * dx_odom;
    vel_x = std::max(std::min(vel_x, max_vel_x), min_vel_x);
    double dyaw = atan2(dy_odom, dx_odom);
    if (dist < rotate_dist_threshold)
    {
        // vel_yaw = 0.0;
        dyaw = goal_yaw - yaw;
        if (dyaw > M_PI)
        {
            dyaw -= 2 * M_PI;
        }
        else if (dyaw < -M_PI)
        {
            dyaw += 2 * M_PI;
        }
    }

    double vel_yaw = proportional_gain_yaw * dyaw;

    if (dist < goal_tolerance && std::abs(dyaw) < yaw_tolerance * M_PI / 180.0)
    {
        path_queue.pop();
        return cmd;
        // goal_reached_msg.data = true;
    }
    else
    {
        // goal_reached_msg.data = false;
    }
    cmd.linear.x() = vel_x;
    cmd.linear.y() = 0.0;
    cmd.linear.z() = 0.0;
    cmd.angular.z() = vel_yaw;
    cmd.angular.y() = 0.0;
    cmd.angular.x() = 0.0;

    // Path_Movement path = readPath();
    // Thread_Safe_Queue<Trajectory_Point> trajectories = readPath();
    // Velocity2d cmd = follow_trajectory(state, path_queue);

    double t = dist / difference_distance;
    double interpolated = (path.value().first.z() * t + path.value().second.z() * (1 - t));
    // std::cout << t << std::endl;

    return cmd;
}
