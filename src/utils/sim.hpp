#pragma once
#include "controllers/linear_controller.hpp"
#include "pid.hpp"
#include "robots/robot.hpp"
#include "types.hpp"
#include <raylib.h>

class Sim_Quadruped : public Robot
{

  public:
    Sim_Quadruped()
    { // horizon_steps, num_samples, dt, temperature

        // Initialize robot state
        pose_state.position = Vector3d(0, 0, 0.5); // Starting position with z=0.5 (standing)
        pose_state.orientation = Eigen::Affine3d::Identity();
        pose_state.velocity.linear = Vector3d::Zero();
        pose_state.velocity.angular = Vector3d::Zero();

        Robot_Config config = {
            .hz = 50,
            .motion_constraints =
                {
                    .max_velocity = 2.0,
                    .max_acceleration = 0.5,
                    .max_deceleration = 0.5,
                    .max_jerk = 0.0,
                },
        };

        PIDGains linear_gains = { 1.2, 0.0, 0.0 };
        PIDController linear_pid(linear_gains);
        linear_pid.output_max = 100.0;
        linear_pid.output_min = 0.0;
        PIDGains angular_gains = { 0.2, 0.0, 0.0 };
        PIDController angular_pid(angular_gains);
        angular_pid.output_max = 10.0;
        angular_pid.output_min = 0.0;

        trajectory_controller = std::make_unique<Linear_Controller>(linear_pid, angular_pid, config.hz);
        trajectory_controller->robot = this;
    }

    ~Sim_Quadruped() = default;

    // Apply a disturbance to the robot (e.g., someone pushing it)
    void
    applyDisturbance(const Eigen::Vector3d &force, const Eigen::Vector3d &torque)
    {
        std::cout << "Applying disturbance: force=" << force.transpose() << ", torque=" << torque.transpose()
                  << std::endl;

        // Simplified disturbance model - directly modify velocity
        pose_state.velocity.linear += force * 0.1; // Scale for reasonable effect
        pose_state.velocity.angular += torque * 0.1;
    }
    void send_velocity_command(Velocity2d &velocity) override;

    Pose_State
    read_state() override
    {
        return pose_state;
    };
};

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
