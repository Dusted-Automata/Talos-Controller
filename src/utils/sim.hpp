#pragma once
#include "controllers/linear_controller.hpp"
#include "pid.hpp"
#include "robots/robot.hpp"
#include "types.hpp"
#include <raylib.h>

class Sim_Display
{
  private:
    Camera2D camera = {};
    Robot &robot;
    std::vector<Ecef_Coord> &waypoints;

    static constexpr int SCREEN_WIDTH = 1000;
    static constexpr int SCREEN_HEIGHT = 1000;
    static constexpr float ROBOT_SIZE = 20.0;
    static constexpr int TARGET_FPS = 500;

    void draw_log_line(int line, const char *format, ...);
    void draw_absolute_grid(Camera2D camera, float gridStep);

  public:
    Sim_Display(Robot &robot, std::vector<Ecef_Coord> &waypoints) : robot(robot), waypoints(waypoints)
    {
        SetTraceLogLevel(LOG_WARNING);
        InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Absolute Coordinate System");
        SetTargetFPS(TARGET_FPS);

        camera.target = { (float)robot.frames.local_frame.pos.x(), (float)robot.frames.local_frame.pos.y() };
        camera.offset = { .x = SCREEN_WIDTH / 2.0f, .y = SCREEN_HEIGHT / 2.0f };
        camera.zoom = 10.0f;
    }

    void display();
    void hud();
    void draw_robot();
};
