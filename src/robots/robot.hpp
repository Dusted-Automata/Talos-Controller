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

template<typename T> void control_loop(T &robot, Linear_Controller &controller);
