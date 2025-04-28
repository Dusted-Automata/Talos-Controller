#pragma once
#include "frame_controller.hpp"
#include "logger.hpp"
#include "path_controller.hpp"
#include "sensor_manager.hpp"
#include "types.hpp"
#include <Eigen/Dense>

class Trajectory_Controller;

class Robot
{
    float hz = 500;

    // MAIN CONTROL THREAD
    // PATH GENERATION THREAD
    // SENSOR PROCESSING THREAD

  public:
    virtual ~Robot() = default;

    Pose_State pose_state;
    std::unique_ptr<Trajectory_Controller> trajectory_controller;
    Frame_Controller frame_controller = {};

    double motion_time = 0;
    Robot_Config config = {};
    Logger logger = {};
    Sensor_Manager sensor_manager = {};
    Path_Controller path_controller = {};

    virtual void send_velocity_command(Velocity2d &cmd) = 0;
    virtual Pose_State read_state() = 0;

    void control_loop();
};
