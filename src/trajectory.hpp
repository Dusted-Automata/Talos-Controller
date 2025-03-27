#pragma once
#include "controller.hpp"
#include "pid.hpp"
#include "types.hpp"
#include <fstream>
#include <vector>

class Trajectory_Controller : public Controller
{

  private:
    Robot_Config config;
    PIDController linear_pid;
    PIDController angular_pid;
    double trajectory_time = 0.0;
    double sampling_rate = 1.0;
    bool added_paths = false;
    std::ofstream traj_file;
    std::ofstream time_file;

    bool (*sendVelocityCommand)(Linear_Velocity &, Angular_Velocity &);
    // bool (*getState)(Robot_State &);

    Trajectory_Point trajectory_turn(Motion_Step step);
    std::vector<Trajectory_Point> trajectory_ramp_up(Motion_Step step);
    Trajectory_Point trajectory_cruise(Motion_Step step);
    std::vector<Trajectory_Point> trajectory_ramp_down(Motion_Step step);
    Trajectory_Point new_trajectory_point(Affine3d &robot_frame, Ecef_Coord next_point,
                                          Vector3d linear, Vector3d angular, double dt);

  public:
    Trajectory_Controller(Robot_Config config, PIDController linear_pid, PIDController angular_pid,
                          double sampling_rate)
        : config(config), linear_pid(linear_pid), angular_pid(angular_pid),
          sampling_rate(sampling_rate)
    {

        std::ofstream traj_file("trajectories", std::ios::trunc);
        std::ofstream time_file("time", std::ios::trunc);
    };

    std::vector<Trajectory_Point> generate_trajectory(Pose_State &state, Ecef_Coord current,
                                                      Ecef_Coord next);
    Pose get_current_pose();
    Velocity2d follow_trajectory(Pose_State &state, Thread_Safe_Queue<Ecef_Coord> &path_queue);

    void path_loop(Thread_Safe_Queue<Ecef_Coord> &path, std::vector<Ecef_Coord> &waypoints);
    // void trajectory_loop(Thread_Safe_Queue<Ecef_Coord> &path_queue);
    void local_replanning();
    Velocity2d get_cmd(Pose_State &state, Thread_Safe_Queue<Ecef_Coord> &path_queue) override;

    Thread_Safe_Queue<Trajectory_Point> trajectories;
};
