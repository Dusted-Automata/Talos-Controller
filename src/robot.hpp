#pragma once
#include "mppi.hpp"
#include "trajectory.hpp"
#include "types.hpp"
#include <Eigen/Dense>

class Robot
{
    int motiontime = 0;
    float hz = 500;
    // Thread_Safe_Queue<Trajectory_Point> trajectory_queue;
    Thread_Safe_Queue<Ecef_Coord> waypoint_queue;
    Robot_Config config = {};
    // void (*control_loop)();
    // MAIN CONTROL THREAD
    // PATH GENERATION THREAD
    // SENSOR PROCESSING THREAD

  public:
    Robot(Trajectory_Controller trajectory_controller, MPPI_Controller mppi_controller)
        : trajectory_controller(trajectory_controller), mppi_controller(mppi_controller)

    {

        pose_state.position = Eigen::Vector3d(0, 0, 0.5); // Starting position with z=0.5 (standing)
        pose_state.orientation = Eigen::Affine3d::Identity();
        pose_state.velocity.linear = Vector3d::Zero();
        pose_state.velocity.angular = Vector3d::Zero();
    }

    Pose_State pose_state;
    virtual ~Robot() = default;
    Thread_Safe_Queue<Trajectory_Point> trajectory_queue;
    Thread_Safe_Queue<Ecef_Coord> path_queue;
    Trajectory_Controller trajectory_controller;
    MPPI_Controller mppi_controller;

    virtual void send_velocity_command(Velocity2d &cmd) = 0;
    virtual void update_state() = 0;
    void control_loop();
    virtual Pose_State read_state() = 0;
    void read_path();
    virtual void read_sensors() = 0;
    void controller_update();
};
