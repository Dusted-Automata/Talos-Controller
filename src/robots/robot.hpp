#pragma once
#include "frames.hpp"
#include "linear_controller.hpp"
#include "logger.hpp"
#include "path_planner.hpp"
#include "robot_path.hpp"
#include "types.hpp"
#include "ublox.hpp"

class Robot;

class Reader
{
  public:
    TCP_Server socket;
    std::thread sensor_thread;
    std::atomic_bool running = false;
    std::array<char, TCP_BUFFER_LENGTH> recv_buf;
    Ring_Buffer<char, TCP_BUFFER_LENGTH * 2> buf;
    Robot *robot;

    bool init(Robot &robot);
    void loop();
};

class Robot
{

  public:
    std::atomic<bool> running = false;
    std::atomic<bool> pause = false;
    Pose_State pose_state;
    Frames frames = {};
    Logger logger = {};
    Robot_Config config = {};
    Ublox ublox = {};
    Path_Planner path = {};
    Heading heading;

    Reader TCP_reader;
    // TCP_Socket out = TCP_Socket("127.0.0.1", 55555);

    bool init();

    virtual void send_velocity_command(Velocity2d &cmd) = 0;
    virtual Pose_State read_state() = 0;
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

        if (!robot.pause) {
            robot.pose_state = robot.read_state();
            // bool update_speed = robot.ublox.update_speed(robot.pose_state.velocity); // Currently blocking!!
            // std::cout << "ublox_update_speed: " << update_speed << std::endl;
            frames_move_in_local_frame(robot.frames, robot.pose_state.velocity, dt);
            update_position(robot.ublox, robot.frames);
            update_heading(robot.ublox, robot.frames, robot.heading);
            robot.logger.savePosesToFile(robot.frames);
            robot.logger.saveTimesToFile(std::chrono::duration<double>(clock::now() - motion_time_start).count());

            // Pose target_waypoint = robot.path.global_path.next();
            Velocity2d cmd = { .linear_vel = Linear_Velocity().setZero(), .angular_vel = Angular_Velocity().setZero() };

            Vector3d global_difference = frames_diff(robot.frames, robot.path.global_path.next().local_point);
            Vector3d local_difference = frames_diff(robot.frames, robot.path.path.next().local_point);
            if (eucledean_xy_norm(global_difference) > robot.config.goal_tolerance_in_meters) {
                cmd = controller.get_cmd(robot.pose_state, global_difference, local_difference, dt);
                // std::cout << "cmd: " << cmd.angular.transpose() << std::endl;
            } else {
                robot.path.global_path.progress(robot.path.path_direction);
            }

            // std::cout << "local_dif: " << local_dif.transpose() << std::endl;
            if (eucledean_xy_norm(local_difference) < robot.config.goal_tolerance_in_meters) {
                controller.motion_profile.reset();
                controller.aligned_to_goal_waypoint = false;
                if (robot.path.path.progress(robot.path.path_direction)) {
                    // Vector3d dif = frames_diff(target_waypoint.local_point, robot.frames);
                    controller.motion_profile.set_setpoint(eucledean_xy_norm(local_difference));

                } else {
                    break;
                }
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
