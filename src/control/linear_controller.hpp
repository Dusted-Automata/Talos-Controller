#pragma once
#include "motion_profile.hpp"
#include "pid.hpp"
#include "trajectory_controller.hpp"
#include "types.hpp"
#include <vector>

class Linear_Controller : public Trajectory_Controller
{

  private:
    LinearPID linear_pid;
    AngularPID angular_pid;
    double trajectory_time = 0.0;
    double goal_tolerance = 0.001; // meters
    double setpoint = 0;

  public:
    Linear_Controller(LinearPID linear_pid, AngularPID angular_pid, Robot_Config config)
        : linear_pid(linear_pid), angular_pid(angular_pid), linear_profile(config.kinematic_constraints.v_max,
                                                                config.kinematic_constraints.a_max,
                                                                config.kinematic_constraints.v_min,
                                                                config.kinematic_constraints.a_min),
          angular_profile(config.kinematic_constraints.omega_max,
              config.kinematic_constraints.alpha_max,
              config.kinematic_constraints.omega_min) {};
    ~Linear_Controller() = default;

    void path_loop(std::vector<Ecef> &waypoints);
    // void trajectory_loop(Thread_Safe_Queue<Ecef_Coord> &path_queue);
    Velocity2d get_cmd() override;
    double get_accel() override;

    Thread_Safe_Queue<Trajectory_Point> trajectories;

    Trapezoidal_Profile linear_profile;
    Trapezoidal_Profile angular_profile;
};
