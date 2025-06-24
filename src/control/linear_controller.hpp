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

  public:
    Linear_Controller(LinearPID linear_pid, AngularPID angular_pid, Robot_Config config)
        : linear_pid(linear_pid), angular_pid(angular_pid), linear_profile(config.kinematic_constraints.v_max,
                                                                config.kinematic_constraints.a_max,
                                                                config.kinematic_constraints.v_min,
                                                                config.kinematic_constraints.a_min) {};
    ~Linear_Controller() = default;

    Velocity2d get_cmd() override;
    double get_accel() override;

    Trapezoidal_Profile linear_profile;
};
