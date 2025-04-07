#pragma once
#include "controller.hpp"
#include "pid.hpp"
#include "types.hpp"
#include <fstream>
#include <vector>

class Linear_Controller : public Trajectory_Controller
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

  public:
    Linear_Controller(Robot_Config config, PIDController linear_pid, PIDController angular_pid,
                      double sampling_rate)
        : config(config), linear_pid(linear_pid), angular_pid(angular_pid),
          sampling_rate(sampling_rate)
    {

        std::ofstream traj_file("poses", std::ios::trunc);
        std::ofstream time_file("times", std::ios::trunc);
    };

    void path_loop(Thread_Safe_Queue<Ecef_Coord> &path, std::vector<Ecef_Coord> &waypoints);
    // void trajectory_loop(Thread_Safe_Queue<Ecef_Coord> &path_queue);
    Velocity2d get_cmd(Pose_State &state, Thread_Safe_Queue<Ecef_Coord> &path_queue) override;

    Thread_Safe_Queue<Trajectory_Point> trajectories;
};
