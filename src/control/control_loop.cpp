
#include "linear_controller.hpp"
#include "path_planner.hpp"
#include "brain.hpp"
#include "timer.h"
#include "robot.hpp"

void
control_loop(Robot &robot, Path_Planner &path_planner, Linear_Controller &controller, Brain &brain)
{
    (void) brain; // TODO: use the Server
    Timer timer;
    timer_init(&timer, robot.config.control_loop_hz);
    while (robot.running) {
        if (!robot.paused) {
            timer_tick(&timer);
            update_pvat(robot.sensor, robot.frames);
            // log() //poses and times

            // TODO change from local_point to an ecef point
            Vector3d to_next_waypoint = frames_diff(robot.frames, path_planner.global_cursor->get_next_waypoint().local_point); //TODO: different function scheme for this.
            to_next_waypoint.z() = 0.0;

            path_planner.global_cursor->advance(to_next_waypoint.norm());
            if (path_planner.global_cursor->at_target_stop(robot.config.goal_tolerance_in_meters)) {
                printf("distance_to_target_stop is below goal_tolerance_in_meters\n");
                // for (u32 i = 1; i < (u32)server.socket.nfds; ++i){
                //     Client client = server.socket.clients[i];
                //     std::string success = "success\n";
                //     tcp_send(client.fd, success.data(), success.length());
                // }
                path_planner.global_cursor->consume_target();
                path_planner.global_cursor->update_target_stop();
                controller.aligned_to_goal_waypoint = false; 
                controller.motion_profile.reset();
                controller.motion_profile.set_setpoint(path_planner.global_cursor->distance_to_target_stop());

            }
            Velocity2d cmd = controller.get_cmd(to_next_waypoint, path_planner.global_cursor->distance_to_target_stop(), timer.dt);
            robot.send_velocity_command(robot.ctx, cmd);
        } else {
            timer_reset(&timer);
        }
    }
}

