#pragma once
#include "frame_controller.hpp"
#include "logger.hpp"
#include "path_controller.hpp"
#include "sensor_manager.hpp"
#include "trajectory_controller.hpp"
#include "types.hpp"
#include <Eigen/Dense>

class Robot
{
    double motion_time = 0;
    float hz = 500;
    Robot_Config config = {};
    Logger logger = {};
    Sensor_Manager sensor_manager = {};
    Path_Controller path_controller = {};

    // MAIN CONTROL THREAD
    // PATH GENERATION THREAD
    // SENSOR PROCESSING THREAD

  public:
    Robot(Trajectory_Controller &controller) : trajectory_controller(controller)
    {
        pose_state.position = Eigen::Vector3d(0, 0, 0.5); // Starting position with z=0.5 (standing)
        pose_state.orientation = Eigen::Affine3d::Identity();
        pose_state.velocity.linear = Vector3d::Zero();
        pose_state.velocity.angular = Vector3d::Zero();
    }
    virtual ~Robot() = default;

    Pose_State pose_state;
    Thread_Safe_Queue<Ecef_Coord> path_queue;
    Trajectory_Controller &trajectory_controller;
    Frame_Controller frame_controller = {};

    virtual void send_velocity_command(Velocity2d &cmd) = 0;
    virtual void update_state() = 0;
    virtual Pose_State read_state() = 0;
    virtual void read_sensors() = 0;

    void control_loop();
    void read_path();
};
