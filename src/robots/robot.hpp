#pragma once
#include "frames.hpp"
#include "logger.hpp"
#include "robot_path.hpp"
#include "types.hpp"
#include "ublox.hpp"

class Trajectory_Controller;

class Robot
{

    // MAIN CONTROL THREAD
    // PATH GENERATION THREAD
    // SENSOR PROCESSING THREAD
    std::thread control_loop_thread;
    std::chrono::steady_clock::time_point motion_time_start;
    std::atomic<bool> running = false;

  public:
    std::atomic<bool> pause = false;
    virtual ~Robot() { shutdown(); };

    Pose_State pose_state;
    std::unique_ptr<Trajectory_Controller> trajectory_controller;
    Frames frames = {};

    Robot_Config config = {};
    Logger logger = {};
    Ublox ublox = {};
    Robot_Path path = {};

    virtual void send_velocity_command(Velocity2d &cmd) = 0;
    virtual Pose_State read_state() = 0;

    void control_loop();
    void start();
    void shutdown();
    bool init();
};
