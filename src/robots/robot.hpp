#pragma once
#include "frames.hpp"
#include "logger.hpp"
#include "path_planner.hpp"
#include "sensor.hpp"
#include "types.hpp"



// Could use a template instead for the ctx. Not sure yet! 
class Robot
{

  public:
    std::atomic<bool> running = false;
    std::atomic<bool> paused = true;
    PVA pva;
    Path_Planner p_planner;
    Frames frames = {};
    Robot_Config config = {};
    // Ublox ublox = {};
    Navigation_Sensor sensor = {};

    bool stop();
    bool pause();
    bool resume();
    void set_target();

    PVA get_PVA();
    void get_path();
    void* ctx;

    LA (*read_pv)(void* ctx) = nullptr;
    void (*send_velocity_command)(void* ctx, Velocity2d &cmd) = nullptr;
    void init(void (*init_ctx)(void*,const Robot* robot));
    void (*deinit)(void* ctx) = nullptr;
};

