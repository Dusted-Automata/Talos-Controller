#include "linear_controller.hpp"
#include "pid.hpp"
#include "raylib.h"
#include "sim.hpp"
#include "transformations.hpp"

class Sim_Quadruped : public Robot
{
    Pose_State sim_pose = {};

  public:
    Sim_Quadruped()
    {
        pose_state.position = Vector3d(0, 0, 0.5); // Starting position with z=0.5 (standing)
        pose_state.orientation = Eigen::Affine3d::Identity();
        pose_state.velocity.linear = Vector3d::Zero();
        pose_state.velocity.angular = Vector3d::Zero();

        config = {
            .control_loop_hz = 500,
            .goal_tolerance_in_meters = 0.05,
            .kinematic_constraints =
            {
                .v_max = 2.5,
                .v_min = -2.0,
                .omega_max = 2.0,
                .omega_min = -2.0,
                .a_max = 1.0,
                .a_min = -1.0,
                .j_max = 0.0,
            },
        };

        Robot::init();
    }

    void
    applyDisturbance(const Eigen::Vector3d &force, const Eigen::Vector3d &torque)
    {
        std::cout << "Applying disturbance: force=" << force.transpose() << ", torque=" << torque.transpose()
                  << std::endl;

        pose_state.velocity.linear += force * 0.1;
        pose_state.velocity.angular += torque * 0.1;
    }
    void
    send_velocity_command(Velocity2d &velocity) override
    {

        sim_pose.velocity = velocity;
        sim_pose.dt = GetFrameTime();
        velocity.linear *= sim_pose.dt;
        velocity.angular *= sim_pose.dt;
    };

    Pose_State
    read_state() override
    {
        return sim_pose;
    };
};

int
main()
{

    Sim_Quadruped robot;

    { // Find out how to extract this.

        double dt = 1.0 / robot.config.control_loop_hz; // TODO change with real dt
        // PIDGains linear_gains = { 0.8, 0.05, 0.15 };
        PIDGains linear_gains = { 1.0, 0.0, 0.0 };
        LinearPID linear_pid(robot.config, linear_gains);
        // PIDGains angular_gains = { 1.0, 1.15, 0.06 };
        PIDGains angular_gains = { 1.0, 0.0, 0.0 };
        AngularPID angular_pid(robot.config, angular_gains);
        Trapezoidal_Profile linear_profile(robot.config.kinematic_constraints.v_max,
            robot.config.kinematic_constraints.a_max, robot.config.kinematic_constraints.v_min,
            robot.config.kinematic_constraints.a_min);
        Linear_Controller traj_controller(linear_gains, angular_gains, linear_profile, robot.config);

        robot.path.path_looping = true;
        robot.path.read_json_latlon("ecef_points.json");
        robot.frames.init(robot.path.path_points_all);

        Sim_Display sim = Sim_Display(robot, robot.path.path_points_all);
        std::jthread sim_thread(&Sim_Display::display, sim);

        while (robot.running) { // Control loop
            while (!robot.pause && robot.running) {
                robot.pose_state = robot.read_state();
                robot.frames.move_in_local_frame(robot.pose_state.velocity, dt);
                robot.logger.savePosesToFile(robot.frames);
                // robot.logger.saveTimesToFile(std::chrono::duration<double>(clock::now() -
                // motion_time_start).count());

                Velocity2d cmd = traj_controller.get_cmd(robot, dt);
                robot.send_velocity_command(cmd);
            }
        }
    }

    CloseWindow();
    return 0;
}
