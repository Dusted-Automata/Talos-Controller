#pragma once
#include "frames.hpp"
#include "logger.hpp"
#include "robot_path.hpp"
#include "types.hpp"

class Robot
{

  public:
    std::atomic<bool> running = false;
    std::atomic<bool> pause = false;
    Pose_State pose_state;
    Frames frames = {};
    Logger logger = {};
    Robot_Path path = {};

    virtual void send_velocity_command(Velocity2d &cmd) = 0;
    virtual Pose_State read_state() = 0;
};

// template<typename Robot, typename Controller>
// void
// update(Robot &robot, Controller &controller, double dt)
// {
//     robot.pose_state = robot.read_state();
//     robot.frames.move_in_local_frame(robot.pose_state.velocity, dt);
//     robot.logger.savePosesToFile(robot.frames);
//     // robot.logger.saveTimesToFile(std::chrono::duration<double>(clock::now() -
//     // motion_time_start).count());
//
//     Velocity2d cmd = controller.get_cmd(robot, dt);
//     robot.send_velocity_command(cmd);
// }
