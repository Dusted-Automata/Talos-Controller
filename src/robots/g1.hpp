#pragma once 
#include "robot.hpp"

#include <chrono>
#include <iostream>
#include <thread>

#include <unitree/robot/g1/loco/g1_loco_api.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>

struct G1 {
    unitree::robot::g1::LocoClient client;
};

void g1_send_velocity_command(void* ctx, Velocity2d &velocity);
void g1_init(void* ctx, const Robot* robot);
