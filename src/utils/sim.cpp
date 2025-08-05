#include "sim.hpp"
#include "cppmap3d.hh"
// #include "mppi.hpp"
#include "json.hpp"
#include "raylib.h"
#include "raymath.h"
#include "transformations.hpp"
#include "ublox.hpp"
#include <stdio.h>
using json = nlohmann::json;

void
Sim_Display::draw_absolute_grid(Camera2D camera, float gridStep)
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

// void
// showcaseTrajectory(Pose_State state, Control_Sequence controls, double dt, float thickness, Color color)
// {
//     Eigen::Matrix3d traj_R = state.orientation.rotation();
//     Ecef traj_pos = state.position;
//     Vector3d to_front = { 1.0, 0.0, 0.0 };
//     to_front = traj_R * to_front;
//     traj_pos += to_front;
//     // traj_pos.y() += 0.5;
//     for (auto control : controls) {
//
//         // Linear_Velocity lv = traj_R * control.linear * dt;
//         Eigen::AngleAxisd yaw_rotation(Eigen::AngleAxisd((control.angular.z() * dt), Vector3d::UnitZ()));
//         traj_R = traj_R * yaw_rotation;
//         // traj_R.normalize();
//         Linear_Velocity lv = traj_R * control.linear * dt;
//
//         DrawLineEx({ (float)traj_pos.x(), (float)-traj_pos.y() },
//             { (float)(traj_pos.x() + lv.x()), (float)(-traj_pos.y() - lv.y()) }, thickness, color);
//         traj_pos = traj_pos + lv;
//     }
// }

void
Sim_Display::draw_robot()
{
    Eigen::Matrix3d R = robot.frames.local_frame.orientation.rotation();
    double yaw = atan2(R(1, 0), R(0, 0));
    Rectangle bot = { static_cast<float>(robot.frames.local_frame.pos.north()),
        static_cast<float>(-robot.frames.local_frame.pos.east()), 1.0, 0.5 };
    Vector2 origin = { bot.width / 2, bot.height / 2 };
    DrawRectanglePro(bot, origin, (float)(-to_degrees(yaw)), RED);

    // Rectangle heading_indicator = { static_cast<float>(robot.frames.local_frame.pos.north()),
    //     static_cast<float>(-robot.frames.local_frame.pos.east() + bot.height), 0.10, 1.0 };
    // Vector2 heading_origin = { (float)robot.frames.local_frame.pos.north(),
    // (float)robot.frames.local_frame.pos.east() }; DrawRectanglePro(heading_indicator, heading_origin,
    //     robot.heading.heading_from_ublox, GREEN);

    // showcaseTrajectory(robot.pose_state, robot.mppi_controller.nominal_controls,
    // GetFrameTime(),
    //                    0.1, ORANGE);
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
        draw_absolute_grid(camera, gridStep);

        for (size_t i = 0; i < path.path.size(); i++) {
            // ENU waypoint = cppmap3d::ecef2enu(path[i].point, robot.frames.local_frame.origin);
            ENU waypoint = path.path[i].local_point;
            DrawCircleV({ (float)waypoint.north(), (float)-waypoint.east() }, 0.5f, GREEN);
        }

        for (size_t i = 0; i < path.global_path.size(); i++) {
            // ENU waypoint = cppmap3d::ecef2enu(path[i].point, robot.frames.local_frame.origin);
            ENU waypoint = path.global_path[i].local_point;
            DrawCircleV({ (float)waypoint.north(), (float)-waypoint.east() }, 0.1f, BLUE);
        }

        ENU waypoint = path.global_path.next().local_point;
        DrawCircleV({ (float)waypoint.north(), (float)-waypoint.east() }, 0.25f, ORANGE);

        draw_robot();

        if (IsKeyPressed(KEY_SPACE)) {

            // Vector3d push = { 1.0, 2.0, 0.0 };
            // ENU fake_measurement = robot.frames.local_frame.pos + push;
            // Ecef ecef = cppmap3d::enu2ecef(fake_measurement, robot.frames.local_frame.origin);
            // LLH llh = cppmap3d::ecef2geodetic(ecef);
            // robot.frames.update_based_on_measurement(llh);
            robot.pause = !robot.pause;
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

            auto nav = robot.ublox.get_latest<Nav_Pvat>(Msg_Type::NAV_PVAT);
            if (nav.has_value()) {
                Nav_Pvat nav_pvat = nav.value();
                Ecef ecef = cppmap3d::geodetic2ecef(nav_pvat.llh);
                json new_entry = {
                    {      "id",                           id },
                    {       "x",                     ecef.x() },
                    {       "y",                     ecef.y() },
                    {       "z",                     ecef.z() },
                    {     "lat", nav_pvat.llh.lat() * RAD2DEG },
                    {     "lon", nav_pvat.llh.lon() * RAD2DEG },
                    {     "alt",           nav_pvat.llh.alt() },
                    { "bearing",         nav_pvat.veh_heading },
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

void
Sim_Display::hud()
{

    { // just logging stuff top left of the screen
        const double currentTime = GetTime();
        const Vector2 mousePos = GetMousePosition();
        const Vector2 mouseWorldPos = GetScreenToWorld2D(mousePos, camera);

        const auto R = robot.frames.local_frame.orientation.rotation();
        // const Ecef ecef_pos = wgsned2ecef_d({ mouseWorldPos.x, -mouseWorldPos.y, 0 },
        // robot.frames.local_frame.origin);
        const Ecef ecef_pos = cppmap3d::ned2ecef({ mouseWorldPos.x, -mouseWorldPos.y, 0 },
            robot.frames.local_frame.origin);

        const double yaw = atan2(R(1, 0), R(0, 0));

        draw_log_line(0, "World: (%.1f, %.1f, %.1f) Zoom: %.2f Time: %.2f", ecef_pos.x(), ecef_pos.y(), ecef_pos.z(),
            camera.zoom, currentTime);

        draw_log_line(1, "X: %.2f %.2f %.2f | Y: %.2f %.2f %.2f | Z: %.2f %.2f %.2f | YAW: %.1f°", R(0, 0), R(0, 1),
            R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2), yaw * RAD2DEG);

        draw_log_line(2, "Local Position: %.2f, %.2f, %.2f", robot.frames.local_frame.pos.north(),
            robot.frames.local_frame.pos.east(), robot.frames.local_frame.pos.down());

        draw_log_line(3, "Global Position: %.2f, %.2f, %.2f", robot.frames.global_frame.pos.x(),
            robot.frames.global_frame.pos.y(), robot.frames.global_frame.pos.z());

        draw_log_line(4, "Velocity: Lin[%.2f, %.2f, %.2f] Ang[%.2f, %.2f, %.2f] Acc:[%.2f)",
            robot.pose_state.velocity.linear_vel.x(), robot.pose_state.velocity.linear_vel.y(),
            robot.pose_state.velocity.linear_vel.z(), robot.pose_state.velocity.angular_vel.x(),
            robot.pose_state.velocity.angular_vel.y(), robot.pose_state.velocity.angular_vel.z());
    }
}
