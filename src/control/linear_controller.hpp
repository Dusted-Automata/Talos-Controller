#pragma once
#include "motion_profile.hpp"
#include "pid.hpp"
#include "types.hpp"

class Linear_Controller
{

  private:
  public:
    Linear_Controller(PIDGains linear_gains, PIDGains angular_gains, Motion_Profile &motion_profile)
        : linear_pid(PID(linear_gains)), angular_pid(PID(angular_gains)), motion_profile(motion_profile)
    {
    }

    Velocity2d get_cmd(Pose_State pose_state, Vector3d diff, Vector3d motion_profile_diff, double dt);
    double goal_tolerance_in_meters;
    PID linear_pid;
    PID angular_pid;
    Motion_Profile &motion_profile;
    bool aligned_to_goal_waypoint = false;
};
