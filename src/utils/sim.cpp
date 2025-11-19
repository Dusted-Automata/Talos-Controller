#include "sim.hpp"
#include "cppmap3d.hh"
// #include "mppi.hpp"
#include "json.hpp"
#include "math.hpp"
#include "raylib.h"
#include "raymath.h"
#include "transformations.hpp"
#include <stdio.h>
using json = nlohmann::json;

void
Sim_Display::draw_absolute_grid(Camera2D camera, float gridStep)
{
    const Vector2 topLeft     = GetScreenToWorld2D({0, 0}, camera);
    const Vector2 screenSize  = { (float)GetScreenWidth(), (float)GetScreenHeight() };
    const Vector2 bottomRight = GetScreenToWorld2D(screenSize, camera);

    // Expand a bit so we cover the edges after rounding
    const float left   = floorf(topLeft.x  / gridStep) * gridStep - gridStep;
    const float right  = floorf(bottomRight.x / gridStep) * gridStep + gridStep;
    const float top    = floorf(topLeft.y  / gridStep) * gridStep - gridStep;
    const float bottom = floorf(bottomRight.y / gridStep) * gridStep + gridStep;

    auto isAxis = [&](float v) {
        // treat values very close to 0 as the axis to avoid float-equality traps
        return fabsf(v) <= (0.5f * gridStep);
    };

    for (float x = left; x <= right; x += gridStep) {
        DrawLineV({ x, top }, { x, bottom }, isAxis(x) ? RED : LIGHTGRAY);
    }

    for (float y = top; y <= bottom; y += gridStep) {
        DrawLineV({ left, y }, { right, y }, isAxis(y) ? RED : LIGHTGRAY);
    }}


void
Sim_Display::draw_robot()
{
    Eigen::Matrix3d R = robot.frames.local_frame.orientation.rotation();
    double yaw = atan2(R(1, 0), R(0, 0));
    Vector2 pos = { static_cast<float>(robot.frames.local_frame.pos.east()),
        static_cast<float>(-robot.frames.local_frame.pos.north()) };
    // if (robot.heading.heading_accuracy > 30.0) {
    //     DrawCircleV(pos, 0.15, RED);
    // } else {
        DrawCircleV(pos, 0.15, SKYBLUE);
    // }
    Vector2 end_pos = { static_cast<float>(0.5), 0 };
    end_pos = Vector2Rotate(end_pos, -yaw);
    end_pos = { end_pos.x + pos.x, end_pos.y + pos.y };
    DrawLineEx(pos, end_pos, 0.05, PURPLE);
}

void
Sim_Display::draw_log_line(int line, const char *format, ...)
{
    char buffer[512];
    va_list args;

    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    DrawText(buffer, 10, 10 + line * 30, 20, BLACK);
}

void
Sim_Display::display()
{
    while (!WindowShouldClose()) {

        Vector2 pos = { static_cast<float>(robot.frames.local_frame.pos.east()),
            static_cast<float>(-robot.frames.local_frame.pos.north()) };
        camera.target = pos;
        camera.offset = { GetScreenWidth() / 2.0f, GetScreenHeight() / 2.0f };


        // if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        //     Vector2 delta = GetMouseDelta();
        //     delta = Vector2Scale(delta, -1.0f / camera.zoom);
        //     camera.target = Vector2Add(camera.target, delta);
        // }

        float wheel = GetMouseWheelMove();
        if (wheel != 0) {
            float scaleFactor = 1.0f + (0.25f * fabsf(wheel));
            camera.zoom *= (wheel > 0) ? scaleFactor : 1.0f / scaleFactor;
            camera.zoom = Clamp(camera.zoom, 0.125f, 254.0f);
        }

        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode2D(camera);

        float gridStep = 1.0f;
        draw_absolute_grid(camera, gridStep);

        for (size_t i = 0; i < path.global_cursor->path->num_waypoints(); i++) {
            // ENU waypoint = cppmap3d::ecef2enu(path[i].point, robot.frames.local_frame.origin);
            ENU waypoint = path.global_cursor->path->waypoint(i).local_point;
            DrawCircleV({ (float)waypoint.east(), (float)-waypoint.north() }, 0.5f, GREEN);
        }
        // for (size_t i = 0; i < path.local_path.size(); i++) {
        //     // ENU waypoint = cppmap3d::ecef2enu(path[i].point, robot.frames.local_frame.origin);
        //     ENU waypoint = path.local_path[i].local_point;
        //     DrawCircleV({ (float)waypoint.east(), (float)-waypoint.north() }, 0.1f, BLUE);
        // }

        // ENU waypoint = path.local_path.next().local_point;
        // DrawCircleV({ (float)waypoint.east(), (float)-waypoint.north() }, 0.25f, ORANGE);

        draw_robot();

        if (IsKeyPressed(KEY_SPACE)) {

            // Vector3d push = { 1.0, 2.0, 0.0 };
            // ENU fake_measurement = robot.frames.local_frame.pos + push;
            // Ecef ecef = cppmap3d::enu2ecef(fake_measurement, robot.frames.local_frame.origin);
            // LLH llh = cppmap3d::ecef2geodetic(ecef);
            // robot.frames.update_based_on_measurement(llh);
            robot.paused = !robot.paused;
        }
        if (IsKeyPressed(KEY_P)) {

            Vector3d push = { 1.0, 2.0, 0.0 };
            ENU fake_measurement = robot.frames.local_frame.pos + push;
            Ecef ecef = cppmap3d::enu2ecef(fake_measurement, robot.frames.local_frame.origin);
            LLH llh = cppmap3d::ecef2geodetic(ecef);
            frames_update_based_on_measurement(robot.frames, llh);
        }

        if (IsKeyPressed(KEY_O)) {
            std::string filename = "new_positions.json";

            // 1. Open the file for reading
            std::ifstream input_file(filename);
            json j;
            int id = 0;

            if (input_file.is_open()) {
                input_file >> j;
                input_file.close();
            } else {
                // If the file doesn't exist or is empty, start with an empty array
                j = json::object();
            }
            if (!j.contains("points") || !j["points"].is_array()) {
                j["points"] = json::array();
            }

            if (!j["points"].empty() && j["points"].back().contains("id")) {
                id = j["points"].back()["id"];
                id += 1; // increment for new entry
            }

            auto nav = robot.sensor.msg;
            if (nav.has_value()) {
                navigation_msg msg = nav.value();
                Ecef ecef = cppmap3d::geodetic2ecef(msg.llh);
                json new_entry = {
                    {      "id",                           id },
                    {       "x",                     ecef.x() },
                    {       "y",                     ecef.y() },
                    {       "z",                     ecef.z() },
                    {     "lat", msg.llh.lat() * RAD2DEG },
                    {     "lon", msg.llh.lon() * RAD2DEG },
                    {     "alt",           msg.llh.alt() },
                    { "heading",         msg.heading_yaw },
                };
                j["points"].push_back(new_entry);
                ecef.x() /= 1000;
                ecef.y() /= 1000;
                ecef.z() /= 1000;
                std::cout << "pushed new Ecef_Point: " << ecef.raw().transpose() << std::endl;
            }

            std::ofstream output_file(filename);
            if (output_file.is_open()) {
                output_file << j.dump(4); // Pretty print with 4-space indentation
                output_file.close();
            } else {
                std::cerr << "Failed to open file for writing\n";
            }
        }

        EndMode2D();

        hud();

        EndDrawing();
    }
    CloseWindow();
}

void
Sim_Display::close()
{
    CloseWindow();
}

// double
// convert_to_positive_radians(double angle)
// {
//
//     if (angle < 0) {
//         return angle + 2 * M_PI;
//     }
//     return angle;
// }
void
Sim_Display::hud()
{

    { // just logging stuff top left of the screen
        const double currentTime = GetTime();
        const Vector2 mousePos = GetMousePosition();
        const Vector2 mouseWorldPos = GetScreenToWorld2D(mousePos, camera);

        // const auto R = robot.frames.local_frame.orientation.rotation();
        const auto R = robot.frames.local_frame.orientation;
        // const Ecef ecef_pos = wgsned2ecef_d({ mouseWorldPos.x, -mouseWorldPos.y, 0 },
        // robot.frames.local_frame.origin);
        const Ecef ecef_pos = cppmap3d::ned2ecef({ mouseWorldPos.x, -mouseWorldPos.y, 0 },
            robot.frames.local_frame.origin);

        const double yaw = convert_to_positive_radians(atan2(R(1, 0), R(0, 0)));

        draw_log_line(0, "World: (%.1f, %.1f, %.1f) Zoom: %.2f Time: %.2f", ecef_pos.x(), ecef_pos.y(), ecef_pos.z(),
            camera.zoom, currentTime);

        draw_log_line(1, "X: %.2f %.2f %.2f | Y: %.2f %.2f %.2f | Z: %.2f %.2f %.2f | YAW: %.1f°", R(0, 0), R(0, 1),
            R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2), yaw * RAD2DEG);

        draw_log_line(2, "Local Position: %.2f, %.2f, %.2f", robot.frames.local_frame.pos.east(),
            robot.frames.local_frame.pos.north(), robot.frames.local_frame.pos.up());

        draw_log_line(3, "Global Position: %.2f, %.2f, %.2f", robot.frames.global_frame.pos.x(),
            robot.frames.global_frame.pos.y(), robot.frames.global_frame.pos.z());

        draw_log_line(4, "Velocity: Lin[%.2f, %.2f, %.2f] Ang[%.2f, %.2f, %.2f] Acc:[%.2f)",
            robot.pva.linear.velocity.x(), robot.pva.linear.velocity.y(),
            robot.pva.linear.velocity.z(), robot.pva.angular.velocity.x(),
            robot.pva.angular.velocity.y(), robot.pva.angular.velocity.z());
    }
}
