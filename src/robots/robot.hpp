#pragma once
#include "timer.h"
#include "frames.hpp"
#include "linear_controller.hpp"
#include "logger.hpp"
#include "path_planner.hpp"
#include "robot_path.hpp"
#include "server.hpp"
#include "types.hpp"
#include "ublox.hpp"


class Robot
{

  public:
    std::atomic<bool> running = false;
    std::atomic<bool> paused = true;
    PVA pva;
    Frames frames = {};
    Robot_Config config = {};
    Ublox ublox = {};

    bool stop();
    bool pause();
    bool resume();
    void set_target();

    PVA get_PVA();
    void get_path();
    LA (*read_pv)() = nullptr;

    virtual void send_velocity_command(Velocity2d &cmd) = 0;
};

template<typename T>
void
control_loop(T &robot, Path_Planner &path_planner, Linear_Controller &controller, Server &server)
{
    Timer timer;
    timer_init(&timer, robot.config.control_loop_hz);
    while (robot.running) {               // Control loop
        if (!robot.paused) {
            timer_tick(&timer);
            {
                {
                    // robot.pva = robot.read_state();
                    LA la = robot.read_pv();
                    robot.pva.linear = la.linear;
                    robot.pva.angular = la.angular;
                    // update_pv(robot.frames);
                    frames_move_in_local_frame(robot.frames, robot.pva.linear, robot.pva.angular, timer.dt);
                    robot.pva.pose.local_point = robot.frames.local_frame.pos;
                    robot.pva.pose.point = robot.frames.global_frame.pos;


                }
                update_position(robot.ublox, robot.frames);
                update_heading(robot.ublox, robot.frames);
            }
            // log() //poses and times


            // Pose target_waypoint = path_planner.global_path.next();
            Velocity2d cmd = { .linear_vel = Linear_Velocity().setZero(), .angular_vel = Angular_Velocity().setZero() };

            Vector3d local_difference = frames_diff(robot.frames, path_planner.local_path.next().local_point);   //TODO: different function scheme for this.
            Vector3d to_next_waypoint = frames_diff(robot.frames, path_planner.global_cursor->get_next_waypoint().local_point); //TODO: different function scheme for this.

            local_difference.z() = 0.0;
            to_next_waypoint.z() = 0.0;

            cmd = controller.get_cmd(robot.pva, to_next_waypoint, path_planner.global_cursor->distance_to_target_stop(), timer.dt);
            // if (local_difference.norm() > robot.config.goal_tolerance_in_meters) {
            //     cmd = controller.get_cmd(robot.pva, local_difference, path_planner.global_cursor->distance_to_target_stop(), dt);
            // } else {
            //     path_planner.local_path.progress(path_planner.path_direction);
            // }

            path_planner.global_cursor->advance(to_next_waypoint.norm());
            if (path_planner.global_cursor->at_target_stop(robot.config.goal_tolerance_in_meters)) {
                printf("distance_to_target_stop is below goal_tolerance_in_meters");
                for (u32 i = 1; i < (u32)server.socket.nfds; ++i){
                    Client client = server.socket.clients[i];
                    std::string success = "success\n";
                    tcp_send(client.fd, success.data(), success.length());
                }
                path_planner.global_cursor->update_target_stop();
                controller.aligned_to_goal_waypoint = false; 
                controller.motion_profile.reset();
                controller.motion_profile.set_setpoint(path_planner.global_cursor->distance_to_target_stop());

            }
            robot.send_velocity_command(cmd);
        } else {
            timer_reset(&timer);
        }
    }
}

