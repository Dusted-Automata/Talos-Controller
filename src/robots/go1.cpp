#include "go1.hpp"
#include "../include/unitree_legged_sdk/unitree_legged_sdk.h"
#include "raylib.h"
#include "raymath.h"
#include "transformations.hpp"
#include "unitree_legged_sdk/comm.h"
#include <chrono>
#include <iostream>
#include <math.h>
#include <stdint.h>
#include <vector>

using steady_clock = std::chrono::steady_clock; // monotonic, no wall‑clock jumps
using ms = std::chrono::milliseconds;
using time_point = steady_clock::time_point;

void
Go1_Quadruped::UDPRecv()
{
    udp.Recv();
}

void
Go1_Quadruped::UDPSend()
{
    udp.Send();
}

UT::HighCmd
defaultCmd()
{
    UT::HighCmd cmd;
    cmd.footRaiseHeight = 0;
    cmd.bodyHeight = 0;
    cmd.euler[0] = 0;
    cmd.euler[1] = 0;
    cmd.euler[2] = 0;
    cmd.velocity[0] = 0.0f;
    cmd.velocity[1] = 0.0f;
    cmd.yawSpeed = 0.0f;
    cmd.reserve = 0;
    return cmd;
}

UT::HighCmd
Go1_Quadruped ::moveCmd(Velocity2d &velocity)
{
    // UT::HighCmd cmd = defaultCmd();
    //
    cmd.mode = 2;
    cmd.gaitType = 1;
    cmd.velocity[0] = (float)velocity.linear.x();
    // cmd.velocity[0] = 0.6;
    cmd.yawSpeed = (float)velocity.angular.z();
    return cmd;
}

void
Go1_Quadruped::send_velocity_command(Velocity2d &velocity)
{
    moveCmd(velocity);
    udp.SetSend(cmd);
    frames.move_in_local_frame(velocity);
};

Pose_State
Go1_Quadruped::read_state()
{
    udp.GetRecv(state);
    Pose_State ps;
    // Robot_State s;
    /*ps.orientation.w() = state.imu.quaternion[0];*/
    /*ps.orientation.x() = state.imu.quaternion[1];*/
    /*ps.orientation.y() = state.imu.quaternion[2];*/
    /*ps.orientation.z() = state.imu.quaternion[3];*/
    // ps.position.x() = state.position[0];
    // ps.position.y() = state.position[1];
    // ps.position.z() = state.position[2];
    ps.velocity.linear.x() = state.velocity[0];
    ps.velocity.linear.y() = state.velocity[1];
    ps.velocity.linear.z() = state.velocity[2];
    // ps.velocity.angular.x() = state.position[0];
    // ps.velocity.angular.y() = state.position[1];
    ps.velocity.angular.z() = state.yawSpeed;

    // s.position = state.position;
    // s.velocity = state.velocity;
    // s.yawSpeed = state.yawSpeed;
    return ps;
};

#define SCREEN_WIDTH 1000
#define SCREEN_HEIGHT 1000
#define ROBOT_SIZE 20.0
void
DrawAbsoluteGrid(Camera2D camera, float gridStep)
{
    Vector2 zero = { 0, 0 };
    Vector2 topLeft = GetScreenToWorld2D(zero, camera);
    Vector2 screen = { SCREEN_WIDTH, SCREEN_HEIGHT };
    Vector2 bottomRight = GetScreenToWorld2D(screen, camera);

    for (float x = floorf(topLeft.x / gridStep) * gridStep; x < bottomRight.x; x += gridStep) {
        Vector2 start = { x, topLeft.y };
        Vector2 end = { x, bottomRight.y };
        DrawLineV(start, end, (x == 0) ? RED : LIGHTGRAY);
    }

    for (float y = floorf(topLeft.y / gridStep) * gridStep; y < bottomRight.y; y += gridStep) {
        Vector2 start = { topLeft.x, y };
        Vector2 end = { bottomRight.x, y };
        DrawLineV(start, end, (y == 0) ? RED : LIGHTGRAY);
    }
}

void
DrawLogLine(int line, const char *format, ...)
{
    char buffer[512];
    va_list args;

    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    DrawText(buffer, 10, 10 + line * 30, 20, BLACK);
}

int
main(void)
{
    std::cout << "Robot level set to: HIGH" << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    std::vector<Ecef_Coord> waypoints = {
        {  4100175.625135626, 476368.7899695045, 4846344.356704135 },
        { 4100209.6729529747, 476361.2681338759, 4846316.478097512 },
        { 4100218.5394949187, 476445.5598077707, 4846300.796185957 },
        {   4100241.72195791, 476441.0557096391, 4846281.753675706 }
    };

    std::vector<Ecef_Coord> waypoints_square = {
        { 0.0, 0.0, 0.0 },
        { 2.0, 0.0, 0.0 },
        { 2.0, 2.0, 0.0 },
        { 0.0, 2.0, 0.0 },
    };

    std::vector<Ecef_Coord> waypoints_circle = {
        {    1.5,    0.0, 0.0 },
        {   1.37,   0.61, 0.0 },
        {  1.004,  1.115, 0.0 },
        {  0.464,  1.427, 0.0 },
        { -0.157,  1.492, 0.0 },
        {  -0.75,  1.299, 0.0 },
        { -1.214,  0.882, 0.0 },
        { -1.467,  0.312, 0.0 },
        { -1.467, -0.312, 0.0 },
        { -1.214, -0.882, 0.0 },
        {  -0.75, -1.299, 0.0 },
        { -0.157, -1.492, 0.0 },
        {  0.464, -1.427, 0.0 },
        {  1.004, -1.115, 0.0 },
        {   1.37,  -0.61, 0.0 },
        {    1.5,   -0.0, 0.0 }
    };

    std::vector<Ecef_Coord> waypoints_eight = {
        {   1.5,  1.5, 0.0 },
        { 2.074, 2.03, 0.0 },
        { 2.561, 2.25, 0.0 },
        { 2.886, 2.03, 0.0 },
        {   3.0,  1.5, 0.0 },
        { 2.886, 0.97, 0.0 },
        { 2.561, 0.75, 0.0 },
        { 2.074, 0.97, 0.0 },
        {   1.5,  1.5, 0.0 },
        { 0.926, 2.03, 0.0 },
        { 0.439, 2.25, 0.0 },
        { 0.114, 2.03, 0.0 },
        {   0.0,  1.5, 0.0 },
        { 0.114, 0.97, 0.0 },
        { 0.439, 0.75, 0.0 },
        { 0.926, 0.97, 0.0 },
        {   1.5,  1.5, 0.0 }
    };

    Go1_Quadruped robot;
    robot.path_controller.path_looping = false;
    robot.path_controller.add_waypoints(waypoints);
    robot.path_controller.start();
    robot.sensor_manager.init();
    robot.frames.init(robot.path_controller.path_points_all.front());

    UT::LoopFunc loop_control("control_loop", (float)(1.0 / robot.hz), 3,
        boost::bind(&Go1_Quadruped::control_loop, &robot));
    UT::LoopFunc loop_udpSend("udp_send", (float)(1.0 / robot.hz), 3, boost::bind(&Go1_Quadruped::UDPRecv, &robot));
    UT::LoopFunc loop_udpRecv("udp_recv", (float)(1.0 / robot.hz), 3, boost::bind(&Go1_Quadruped::UDPSend, &robot));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    {

        SetTraceLogLevel(LOG_WARNING);
        InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Absolute Coordinate System");
        SetTargetFPS(500);

        Camera2D camera = {};
        camera.target = { (float)robot.frames.local_frame.pos.x(), (float)robot.frames.local_frame.pos.y() };
        camera.offset = { .x = SCREEN_WIDTH / 2.0f, .y = SCREEN_HEIGHT / 2.0f };
        camera.zoom = 10.0f;

        while (!WindowShouldClose()) {
            if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
                Vector2 delta = GetMouseDelta();
                delta = Vector2Scale(delta, -1.0f / camera.zoom);
                camera.target = Vector2Add(camera.target, delta);
            }

            float wheel = GetMouseWheelMove();
            if (wheel != 0) {
                Vector2 mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), camera);
                camera.offset = GetMousePosition();
                camera.target = mouseWorldPos;

                float scaleFactor = 1.0f + (0.25f * fabsf(wheel));
                camera.zoom *= (wheel > 0) ? scaleFactor : 1.0f / scaleFactor;
                camera.zoom = Clamp(camera.zoom, 0.125f, 254.0f);
            }

            BeginDrawing();
            ClearBackground(RAYWHITE);

            BeginMode2D(camera);

            float gridStep = 5.0f;
            DrawAbsoluteGrid(camera, gridStep);

            for (size_t i = 0; i < waypoints.size(); i++) {
                Vector3d waypoint = wgsecef2ned_d(waypoints[i], robot.frames.local_frame.origin);
                DrawCircle((int)waypoint.x(), (int)-waypoint.y(), 0.8f, BLUE);
            }

            Eigen::Matrix3d R = robot.frames.local_frame.orientation.rotation();
            double yaw = atan2(R(1, 0), R(0, 0));
            Rectangle bot = { static_cast<float>(robot.frames.local_frame.pos.x()),
                static_cast<float>(-robot.frames.local_frame.pos.y()), 2.0, 1.0 };
            Vector2 origin = { 2.0 / 2.0, 1.0 / 2.0 };
            DrawRectanglePro(bot, origin, (float)(-(yaw * 180.0) / M_PI), RED);

            if (IsKeyPressed(KEY_SPACE)) {

                Vector3d push = { 1.0, 2.0, 0.0 };
                Vector3d fake_measurement = robot.frames.local_frame.pos + push;
                Ecef_Coord ecef = wgsned2ecef_d(fake_measurement, robot.frames.local_frame.origin);
                LLH llh = wgsecef2llh(ecef);
                robot.frame_controller.update_based_on_measurement(llh[0], llh[1], llh[2]);
            }

            EndMode2D();

            { // just logging stuff top left of the screen
                const double currentTime = GetTime();
                const Vector2 mousePos = GetMousePosition();
                const Vector2 mouseWorldPos = GetScreenToWorld2D(mousePos, camera);

                const auto R = robot.frames.local_frame.orientation.rotation();
                const Ecef_Coord ecef_pos = wgsned2ecef_d({ mouseWorldPos.x, -mouseWorldPos.y, 0 },
                    robot.frames.local_frame.origin);

                const double yaw = atan2(R(1, 0), R(0, 0));

                DrawLogLine(0, "World: (%.1f, %.1f, %.1f) Zoom: %.2f Time: %.2f", ecef_pos.x(), ecef_pos.y(),
                    ecef_pos.z(), camera.zoom, currentTime);

                DrawLogLine(1, "X: %.2f %.2f %.2f | Y: %.2f %.2f %.2f | Z: %.2f %.2f %.2f | YAW: %.1f°", R(0, 0),
                    R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2), yaw * RAD2DEG);

                DrawLogLine(2, "Local Position: %.2f, %.2f, %.2f", robot.frames.local_frame.pos.x(),
                    robot.frames.local_frame.pos.y(), robot.frames.local_frame.pos.z());

                DrawLogLine(3, "Global Position: %.2f, %.2f, %.2f", robot.frames.global_frame.pos.x(),
                    robot.frames.global_frame.pos.y(), robot.frames.global_frame.pos.z());

                DrawLogLine(4, "Velocity: Lin[%.2f, %.2f, %.2f] Ang[%.2f, %.2f, %.2f]",
                    robot.pose_state.velocity.linear.x(), robot.pose_state.velocity.linear.y(),
                    robot.pose_state.velocity.linear.z(), robot.pose_state.velocity.angular.x(),
                    robot.pose_state.velocity.angular.y(), robot.pose_state.velocity.angular.z());
            }

            EndDrawing();
        }

        CloseWindow();
    }

    return 0;
}
