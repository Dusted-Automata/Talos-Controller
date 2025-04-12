#include "../src/mppi.hpp"
#include "../src/robot.hpp"
#include "../src/trajectory_controller.hpp"
#include "../src/types.hpp"
#include "cppmap3d.hh"
#include "linear_controller.hpp"
#include "pid.hpp"
#include "raylib.h"
#include "raymath.h"
#include "transformations.hpp"
#include <iostream>
#include <stdio.h>
#include <thread>

#define SCREEN_WIDTH 1000
#define SCREEN_HEIGHT 1000
#define ROBOT_SIZE 20.0

void worker_function(std::function<void()> callback, int period_ms)
{
    while (1)
    {
        callback();
        std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
    }
}

void DrawAbsoluteGrid(Camera2D camera, float gridStep)
{
    Vector2 zero = {0, 0};
    Vector2 topLeft = GetScreenToWorld2D(zero, camera);
    Vector2 screen = {SCREEN_WIDTH, SCREEN_HEIGHT};
    Vector2 bottomRight = GetScreenToWorld2D(screen, camera);

    for (float x = floorf(topLeft.x / gridStep) * gridStep; x < bottomRight.x; x += gridStep)
    {
        Vector2 start = {x, topLeft.y};
        Vector2 end = {x, bottomRight.y};
        DrawLineV(start, end, (x == 0) ? RED : LIGHTGRAY);
    }

    for (float y = floorf(topLeft.y / gridStep) * gridStep; y < bottomRight.y; y += gridStep)
    {
        Vector2 start = {topLeft.x, y};
        Vector2 end = {bottomRight.x, y};
        DrawLineV(start, end, (y == 0) ? RED : LIGHTGRAY);
    }
}

class Sim_Quadruped : public Robot
{

  public:
    Sim_Quadruped(Trajectory_Controller &controller) : Robot(controller)
    { // horizon_steps, num_samples, dt, temperature

        // Initialize robot state
        pose_state.position = Vector3d(0, 0, 0.5); // Starting position with z=0.5 (standing)
        pose_state.orientation = Eigen::Affine3d::Identity();
        pose_state.velocity.linear = Vector3d::Zero();
        pose_state.velocity.angular = Vector3d::Zero();
    }

    ~Sim_Quadruped() = default;

    // Apply a disturbance to the robot (e.g., someone pushing it)
    void applyDisturbance(const Eigen::Vector3d &force, const Eigen::Vector3d &torque)
    {
        std::cout << "Applying disturbance: force=" << force.transpose()
                  << ", torque=" << torque.transpose() << std::endl;

        // Simplified disturbance model - directly modify velocity
        pose_state.velocity.linear += force * 0.1; // Scale for reasonable effect
        pose_state.velocity.angular += torque * 0.1;
    }

    void send_velocity_command(Velocity2d &velocity) override
    {
        pose_state.velocity = velocity;
        pose_state.dt = GetFrameTime();
        velocity.linear *= pose_state.dt;
        velocity.angular *= pose_state.dt;
        frame_controller.move_in_local_frame(velocity);
    };

    void read_sensors() override{};
    void update_state() override{};

    Pose_State read_state() override { return pose_state; };
};

void showcaseTrajectory(Pose_State state, Control_Sequence controls, double dt, double thickness,
                        Color color)
{
    Eigen::Matrix3d traj_R = state.orientation.rotation();
    Ecef_Coord traj_pos = state.position;
    Vector3d to_front = {1.0, 0.0, 0.0};
    to_front = traj_R * to_front;
    traj_pos += to_front;
    // traj_pos.y() += 0.5;
    for (auto control : controls)
    {

        // Linear_Velocity lv = traj_R * control.linear * dt;
        Eigen::AngleAxisd yaw_rotation(
            Eigen::AngleAxisd((control.angular.z() * dt), Vector3d::UnitZ()));
        traj_R = traj_R * yaw_rotation;
        // traj_R.normalize();
        Linear_Velocity lv = traj_R * control.linear * dt;

        DrawLineEx({(float)traj_pos.x(), (float)-traj_pos.y()},
                   {(float)(traj_pos.x() + lv.x()), (float)(-traj_pos.y() - lv.y())}, thickness,
                   color);
        traj_pos = traj_pos + lv;
    }
}

void simbot_linear(Sim_Quadruped &robot)
{

    robot.control_loop();

    {
        Eigen::Matrix3d R = robot.frame_controller.local_frame.orientation.rotation();
        double yaw = atan2(R(1, 0), R(0, 0));
        Rectangle bot = {static_cast<float>(robot.frame_controller.local_frame.pos.x()),
                         static_cast<float>(-robot.frame_controller.local_frame.pos.y()), 2.0, 1.0};
        Vector2 origin = {2.0 / 2.0, 1.0 / 2.0};
        DrawRectanglePro(bot, origin, (-(yaw * 180) / M_PI), RED);

        // showcaseTrajectory(robot.pose_state, robot.mppi_controller.nominal_controls,
        // GetFrameTime(),
        //                    0.1, ORANGE);
    }
}

int main()
{
    InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Absolute Coordinate System");
    SetTargetFPS(500);

    float dt = 0;

    std::vector<Ecef_Coord> waypoints = {{4100175.625135626, 476368.7899695045, 4846344.356704135},
                                         {4100209.6729529747, 476361.2681338759, 4846316.478097512},
                                         {4100218.5394949187, 476445.5598077707, 4846300.796185957},
                                         {4100241.72195791, 476441.0557096391, 4846281.753675706}};

    // std::vector<Ecef_Coord> waypoints = {{0.0, 0.0, 0.0},    {8.5, 10.5, 0.0},   {10.0, 10.0,
    // 0.0},
    //                                      {21.0, -14.0, 0.0}, {20.0, -15.0, 0.0}, {0.0, -10,
    //                                      0.0}};

    // std::vector<Ecef_Coord> waypoints = {{1.5, 1.5, 0.0},    {2.074, 2.03, 0.0}, {2.561, 2.25,
    // 0.0},
    //                                      {2.886, 2.03, 0.0}, {3.0, 1.5, 0.0},    {2.886, 0.97,
    //                                      0.0}, {2.561, 0.75, 0.0}, {2.074, 0.97, 0.0}, {1.5, 1.5,
    //                                      0.0}, {0.926, 2.03, 0.0}, {0.439, 2.25, 0.0},
    //                                      {0.114, 2.03, 0.0}, {0.0, 1.5, 0.0},    {0.114, 0.97,
    //                                      0.0}, {0.439, 0.75, 0.0}, {0.926, 0.97, 0.0}, {1.5, 1.5,
    //                                      0.0}};

    // std::vector<Ecef_Coord> waypoints = {
    //     {1.5, 0.0, 0.0},       {1.37, 0.61, 0.0},     {1.004, 1.115, 0.0},  {0.464, 1.427, 0.0},
    //     {-0.157, 1.492, 0.0},  {-0.75, 1.299, 0.0},   {-1.214, 0.882, 0.0}, {-1.467, 0.312, 0.0},
    //     {-1.467, -0.312, 0.0}, {-1.214, -0.882, 0.0}, {-0.75, -1.299, 0.0}, {-0.157, -1.492,
    //     0.0}, {0.464, -1.427, 0.0},  {1.004, -1.115, 0.0},  {1.37, -0.61, 0.0},   {1.5, -0.0,
    //     0.0}};

    Robot_Config config = {
        .hz = 50,
        .motion_constraints =
            {
                .max_velocity = 2.0,
                .min_velocity = 0.1,
                .max_acceleration = 0.5,
                .max_deceleration = 0.5,
                .max_jerk = 0.0,
            },
    };

    PIDGains linear_gains = {1.2, 0.0, 0.0};
    PIDController linear_pid(linear_gains);
    linear_pid.output_max = 100.0;
    linear_pid.output_min = 0.0;
    PIDGains angular_gains = {0.2, 0.0, 0.0};
    PIDController angular_pid(angular_gains);
    angular_pid.output_max = 10.0;
    angular_pid.output_min = 0.0;

    Linear_Controller l_c(config, linear_pid, angular_pid, config.hz);

    Sim_Quadruped robot(l_c);

    robot.frame_controller.local_frame.origin = waypoints[0];
    LLH llh = wgsecef2llh(waypoints[0]);
    Eigen::Matrix3d M = wgs_ecef2ned_matrix(llh);
    double theta = atan2(llh.y(), llh.x());
    Eigen::AngleAxisd rot_yaw(theta, Vector3d::UnitZ());
    robot.frame_controller.local_frame.orientation = rot_yaw.toRotationMatrix();
    std::cout << robot.frame_controller.local_frame.orientation.matrix() << std::endl;
    robot.frame_controller.global_frame.pos = waypoints[0];
    robot.frame_controller.global_frame.orientation = M;
    robot.frame_controller.global_frame.orientation.rotate(
        Eigen::AngleAxisd(M_PI / 19, -Vector3d::UnitY()));
    robot.frame_controller.global_frame.orientation.rotate(
        Eigen::AngleAxisd(M_PI / 2, -Vector3d::UnitZ()));
    robot.frame_controller.global_frame.orientation.rotate(
        Eigen::AngleAxisd(M_PI, Vector3d::UnitY()));
    robot.frame_controller.global_frame.orientation.rotate(
        Eigen::AngleAxisd(M_PI / 50, Vector3d::UnitY()));
    /*std::cout << robot.frame_controller.global_frame.orientation.rotation() << std::endl;*/
    /*robot.frame_controller.global_frame.orientation.rotate(Eigen::AngleAxisd(-1,
     * Vector3d::UnitY()));*/
    /*robot.frame_controller.global_frame.orientation.rotate(Eigen::AngleAxisd(-1,
     * Vector3d::UnitY()));*/

    std::function<void()> bound_path_loop = std::bind(
        &Linear_Controller::path_loop, &l_c, std::ref(robot.path_queue), std::ref(waypoints));

    std::thread path_loop = std::thread(worker_function, bound_path_loop, 30);
    path_loop.detach();

    Camera2D camera = {.target = {(float)robot.frame_controller.local_frame.pos.x(),
                                  (float)robot.frame_controller.local_frame.pos.y()}};
    camera.offset = (Vector2){SCREEN_WIDTH / 2.0f, SCREEN_HEIGHT / 2.0f};
    camera.zoom = 10.0f;
    int index = 0;

    while (!WindowShouldClose())
    {
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
        {
            Vector2 delta = GetMouseDelta();
            delta = Vector2Scale(delta, -1.0f / camera.zoom);
            camera.target = Vector2Add(camera.target, delta);
        }

        float wheel = GetMouseWheelMove();
        if (wheel != 0)
        {
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

        for (int i = 0; i < waypoints.size(); i++)
        {
            // TODO: Refactor into own vector or something. No need to convert every frame.
            Vector3d waypoint =
                wgsecef2ned_d(waypoints[i], robot.frame_controller.local_frame.origin);
            DrawCircle(waypoint.x(), -waypoint.y(), 0.8, BLUE);
        }

        simbot_linear(robot);

        EndMode2D();

        Vector2 mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), camera);
        char coordText[500];
        Ecef_Coord ecef_pos = wgsned2ecef_d({mouseWorldPos.x, -mouseWorldPos.y, 0},
                                            robot.frame_controller.local_frame.origin);
        sprintf(coordText, "World: (%.1f, %.1f, %.1f) Zoom: %.2f Time: %f", ecef_pos.x(),
                ecef_pos.y(), ecef_pos.z(), camera.zoom, dt);
        DrawText(coordText, 10, 10, 20, BLACK);
        auto R = robot.frame_controller.local_frame.orientation.rotation();
        char orientationText[500];
        sprintf(orientationText,
                "X: %.2f %.2f %.2f - Y: %.2f %.2f %.2f - Z: %.2f %.2f %.2f - YAW: %.2f", R(0, 0),
                R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2),
                atan2(R(1, 0), R(0, 0)));
        DrawText(orientationText, 10, 40, 20, BLACK);
        char localPositionText[500];
        sprintf(localPositionText, "local position: %.2f %.2f %.2f ",
                robot.frame_controller.local_frame.pos.x(),
                robot.frame_controller.local_frame.pos.y(),
                robot.frame_controller.local_frame.pos.z());
        DrawText(localPositionText, 10, 70, 20, BLACK);
        char globalPositionText[500];
        sprintf(globalPositionText, "global position: %.2f %.2f %.2f ",
                robot.frame_controller.global_frame.pos.x(),
                robot.frame_controller.global_frame.pos.y(),
                robot.frame_controller.global_frame.pos.z());
        DrawText(globalPositionText, 10, 100, 20, BLACK);
        char velocityText[500];
        sprintf(velocityText, "velocity: %.2f %.2f %.2f - %.2f %.2f %.2f",
                robot.pose_state.velocity.linear.x(), robot.pose_state.velocity.linear.y(),
                robot.pose_state.velocity.linear.z(), robot.pose_state.velocity.angular.x(),
                robot.pose_state.velocity.angular.y(), robot.pose_state.velocity.angular.z());
        DrawText(velocityText, 10, 130, 20, BLACK);

        EndDrawing();
        dt = GetTime();
    }

    CloseWindow();
    return 0;
}
