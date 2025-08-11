#pragma once
#include "frames.hpp"
#include "logger.hpp"
#include "path_planner.hpp"
#include "robot_path.hpp"
#include "types.hpp"
#include "ublox.hpp"

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
    // Robot_Path path = {};
    Path_Planner path = {};
    Heading heading;

    TCP_Socket in = TCP_Socket("127.0.0.1", 55555);
    TCP_Socket out = TCP_Socket("127.0.0.1", 55555);

    bool init();

    virtual void send_velocity_command(Velocity2d &cmd) = 0;
    virtual Pose_State read_state() = 0;
};
