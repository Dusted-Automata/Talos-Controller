#pragma once
#include "frames.hpp"
#include "linear_controller.hpp"
#include "logger.hpp"
#include "path_planner.hpp"
#include "robot_path.hpp"
#include "types.hpp"
#include "ublox.hpp"

class Robot;

class Server
{
  public:
    std::thread socket_thread;
    std::thread msg_parse_thread;
    TCP_Server socket;
    Robot *robot;
    std::vector<int> subscribers;

    bool init(Robot &robot);
    void loop();
};

class Robot
{

  public:
    std::atomic<bool> running = false;
    std::atomic<bool> paused = true;
    PVA pva;
    Frames frames = {};
    Logger logger = {};
    Robot_Config config = {};
    Ublox ublox = {};
    Path_Planner path = {};

    Server server;
    // TCP_Socket out = TCP_Socket("127.0.0.1", 55555);

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
control_loop(T &robot, Linear_Controller &controller)
{

    using clock = std::chrono::steady_clock;
    auto next = clock::now();
    auto previous_time = clock::now();
    auto motion_time_start = clock::now();
    std::chrono::milliseconds period(1000 / robot.config.control_loop_hz);

    while (robot.running) {               // Control loop
        auto current_time = clock::now(); // Current iteration time
        std::chrono::duration<double> elapsed = current_time - previous_time;
        double dt = elapsed.count(); // `dt` in seconds
        previous_time = current_time;

        if (!robot.paused) {
            Path_Cursor gc = *robot.path.global_cursor;


            // if (!gc.target_stop_idx.has_value()) {
            //     printf("target_@top_number is NULLOPT\n");
            //     break;
            // }
            // if (!gc.path->next_stop_index(gc.target_stop_idx.value(), gc.dir).has_value()) {
            //     printf("next_stop_index GONNA RETURN NULLOPT\n");
            //     break;
            // };
            //
            // // printf("next_stop_index: %zu\n",
            // //        gc.path->next_stop_index(gc.target_stop_idx.value(), gc.dir).value()
            // //        );
            // if (!gc.path->next_stop_waypoint(gc.current_waypoint, gc.dir).has_value()) {
            //     printf("next_stop_waypoint GONNA RETURN NULLOPT\n");
            //     break;
            // }
            // printf("num_stops: %zu | next_stop_index: %zu | next_stop_waypoint: %zu\n",
            //        gc.path->num_stops(),
            //        gc.path->next_stop_index(gc.target_stop_idx.value(), gc.dir).value(),
            //        gc.path->next_stop_waypoint(gc.target_stop_waypoint, gc.dir).value()
            //        // gc.path->next_stop_number(gc.target_stop_waypoint, gc.dir).value()
            //     );
            // bool update_speed = robot.ublox.update_speed(robot.pose_state.velocity); // Currently blocking!!
            {
                {
                    // robot.pva = robot.read_state();
                    LA la = robot.read_pv();
                    robot.pva.linear = la.linear;
                    robot.pva.angular = la.angular;
                    // update_pv(robot.frames);
                    frames_move_in_local_frame(robot.frames, robot.pva.linear, robot.pva.angular, dt);
                    robot.pva.pose.local_point = robot.frames.local_frame.pos;
                    robot.pva.pose.point = robot.frames.global_frame.pos;


                    PVA pva = robot.get_PVA();
                    json pva_j = {
                        {"position", {"x", pva.pose.point.x(), "y", pva.pose.point.y(), "z", pva.pose.point.z()}},
                        {"velocity", 
                            {
                                "linear", 
                                {
                                    "x", pva.linear.velocity.x(),
                                    "y", pva.linear.velocity.y(),
                                    "z", pva.linear.velocity.z()
                                },
                                "angular", 
                                {
                                    "x", pva.angular.velocity.x(),
                                    "y", pva.angular.velocity.y(),
                                    "z", pva.angular.velocity.z()
                                },

                            },
                        },
                        {"acceleration", 
                            {
                                "linear", 
                                {
                                    "x", pva.linear.acceleration.x(),
                                    "y", pva.linear.acceleration.y(),
                                    "z", pva.linear.acceleration.z()
                                },
                                "angular", 
                                {
                                    "x", pva.angular.acceleration.x(),
                                    "y", pva.angular.acceleration.y(),
                                    "z", pva.angular.acceleration.z()
                                },

                            },
                        },
                    };

                    std::string send_pva = pva_j.dump(4);

                    for (u32 i = 1; i < (u32)robot.server.socket.nfds; ++i){
                        Client client = robot.server.socket.clients[i];
                        tcp_send(client.fd, send_pva.data(), send_pva.length());
                    }
                }
                update_position(robot.ublox, robot.frames);
                update_heading(robot.ublox, robot.frames);
            }

            robot.logger.savePosesToFile(robot.frames);
            robot.logger.saveTimesToFile(std::chrono::duration<double>(clock::now() - motion_time_start).count());

            // Pose target_waypoint = robot.path.global_path.next();
            Velocity2d cmd = { .linear_vel = Linear_Velocity().setZero(), .angular_vel = Angular_Velocity().setZero() };

            Vector3d local_difference = frames_diff(robot.frames, robot.path.local_path.next().local_point);   //TODO: different function scheme for this.
            Vector3d to_next_waypoint = frames_diff(robot.frames, robot.path.global_cursor->get_next_waypoint().local_point); //TODO: different function scheme for this.

            printf("waypoint: %d | target_stop: %d | distance_to_next_waypoint: %f | distance_to_target_stop: %f\n",
                   gc.next_waypoint,
                   gc.target_stop_waypoint,
                   gc.distance_to_next_waypoint(), 
                   gc.distance_to_target_stop()
               );
            local_difference.z() = 0.0;
            to_next_waypoint.z() = 0.0;

            if (local_difference.norm() > robot.config.goal_tolerance_in_meters) {
                cmd = controller.get_cmd(robot.pva, local_difference, robot.path.global_cursor->distance_to_target_stop(), dt);
            } else {
                robot.path.local_path.progress(robot.path.path_direction);
            }

            robot.path.global_cursor->advance(to_next_waypoint.norm());
            // WARN:
            // distance_to_target_stop requires the current waypoint to be calculated, which gets
            // updated in the advance as soon as we are under the tolerance. 
            // if you update the target_idx first then that causes the robot to ask for the next
            // stop target by using the current waypoint, which then shows this as the next one
            // since the waypoint is not driven to yet.
            if (robot.path.global_cursor->distance_to_target_stop() < robot.config.goal_tolerance_in_meters + 0.1) {
                printf("distance_to_target_stop is below goal_tolerance_in_meters");
                for (u32 i = 1; i < (u32)robot.server.socket.nfds; ++i){
                    Client client = robot.server.socket.clients[i];
                    std::string success = "success\n";
                    tcp_send(client.fd, success.data(), success.length());
                }
                robot.path.global_cursor->update_target_stop();
                controller.aligned_to_goal_waypoint = true; //TODO: remove the aligned part
                controller.motion_profile.reset();
                controller.motion_profile.set_setpoint(robot.path.global_cursor->distance_to_target_stop());

            }
            robot.send_velocity_command(cmd);
        }

        next += period;
        std::this_thread::sleep_until(next);

        if (clock::now() > next + period) {
            std::cerr << "control-loop overrun" << std::endl;
            next = clock::now();
            previous_time = next;
        }
    }
}

bool server_init(Server &server, Robot &r);
