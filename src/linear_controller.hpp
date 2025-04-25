#pragma once
#include "pid.hpp"
#include "trajectory_controller.hpp"
#include "types.hpp"
#include <vector>

class Linear_Controller : public Trajectory_Controller
{

  private:
    PIDController linear_pid;
    PIDController angular_pid;
    double trajectory_time = 0.0;
    double sampling_rate = 1.0;
    bool added_paths = false;

  public:
    Linear_Controller(PIDController linear_pid, PIDController angular_pid, double sampling_rate)
        : linear_pid(linear_pid), angular_pid(angular_pid), sampling_rate(sampling_rate){};

    void path_loop(std::vector<Ecef_Coord> &waypoints);
    // void trajectory_loop(Thread_Safe_Queue<Ecef_Coord> &path_queue);
    Velocity2d get_cmd() override;

    Thread_Safe_Queue<Trajectory_Point> trajectories;
};
