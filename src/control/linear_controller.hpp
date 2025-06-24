#pragma once
#include "motion_profile.hpp"
#include "pid.hpp"
#include "trajectory_controller.hpp"
#include "types.hpp"

class Linear_Controller
{

  private:
    LinearPID linear_pid;
    AngularPID angular_pid;
    Motion_Profile &motion_profile;

  public:
    Linear_Controller(
        PIDGains linear_gains, PIDGains angular_gains, Motion_Profile &motion_profile, Robot_Config config)
        : linear_pid(LinearPID(config, linear_gains)), angular_pid(AngularPID(config, angular_gains)),
          motion_profile(motion_profile)
    {
    }

    static Velocity2d calculate_cmd(Robot &robot, Motion_Profile &motion_profile, double dt);
    Velocity2d get_cmd(Robot &robot, double dt);
};
