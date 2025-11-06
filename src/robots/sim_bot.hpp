#pragma once
#include "robot.hpp"
#include "types.hpp"

struct Sim_Bot 
{
    LA sim_velocity = {};
    double inertia = 0.02;
    double damping = 0.05;    
    double angular_velocity = 0.0;    // ω
    double dt = 0.01;
    double turn_left_constraint = 0.1;
    double turn_right_constraint = -0.1;
};

void sim_send_velocity_command(void* ctx, Velocity2d &velocity);
LA sim_read_state(void* ctx);
void sim_init(void* ctx, const Robot* robot);
