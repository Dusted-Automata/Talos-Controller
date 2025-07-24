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

    Robot_Config config = {};
    Logger logger = {};
    Robot_Path path = {};

    virtual void send_velocity_command(Velocity2d &cmd) = 0;
    virtual Pose_State read_state() = 0;
};
