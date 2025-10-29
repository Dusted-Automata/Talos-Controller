#pragma once
#include "frames.hpp"
#include "logger.hpp"
#include "types.hpp"
#include "ublox.hpp"


class Robot
{

  public:
    std::atomic<bool> running = false;
    std::atomic<bool> paused = true;
    PVA pva;
    Frames frames = {};
    Robot_Config config = {};
    Ublox ublox = {};

    bool stop();
    bool pause();
    bool resume();
    void set_target();

    PVA get_PVA();
    void get_path();
    LA (*read_pv)() = nullptr;

    virtual void send_velocity_command(Velocity2d &cmd) = 0;
};

