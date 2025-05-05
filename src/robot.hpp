#pragma once
#include "frame_controller.hpp"
#include "logger.hpp"
#include "path_controller.hpp"
#include "sensor_manager.hpp"
#include "types.hpp"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wextra"
#pragma GCC diagnostic ignored "-Wconversion"
#include <Eigen/Dense>
#pragma GCC diagnostic pop

class Trajectory_Controller;

class Robot
{

    // MAIN CONTROL THREAD
    std::thread control_loop_thread;
    // PATH GENERATION THREAD
    // SENSOR PROCESSING THREAD

  public:
    virtual ~Robot() = default;

    Pose_State pose_state;
    std::unique_ptr<Trajectory_Controller> trajectory_controller;
    Frame_Controller frame_controller = {};

    double hz = 500;
    double motion_time = 0;
    Robot_Config config = {};
    Logger logger = {};
    Sensor_Manager sensor_manager = {};
    Path_Controller path_controller = {};

    virtual void send_velocity_command(Velocity2d &cmd) = 0;
    virtual Pose_State read_state() = 0;

    void control_loop();
    void start();
};
