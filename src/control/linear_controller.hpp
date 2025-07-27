#pragma once
#include "motion_profile.hpp"
#include "pid.hpp"
#include "types.hpp"

class Linear_Controller
{

  private:
  public:
    Linear_Controller(
        PIDGains linear_gains, PIDGains angular_gains, Motion_Profile &motion_profile, Robot_Config config)
        : linear_pid(LinearPID(config, linear_gains)), angular_pid(AngularPID(config, angular_gains)),
          motion_profile(motion_profile)
    {
    }

    Velocity2d get_cmd(Pose_State pose_state, Vector3d diff, double dt);
    double goal_tolerance_in_meters;
    LinearPID linear_pid;
    AngularPID angular_pid;
    Motion_Profile &motion_profile;
};
