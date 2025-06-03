#include "linear_controller.hpp"
#include "pid.hpp"
#include "raylib.h"
#include "sim.hpp"
#include "transformations.hpp"

class Sim_Quadruped : public Robot
{

  public:
    Sim_Quadruped()
    {
        pose_state.position = Vector3d(0, 0, 0.5); // Starting position with z=0.5 (standing)
        pose_state.orientation = Eigen::Affine3d::Identity();
        pose_state.velocity.linear = Vector3d::Zero();
        pose_state.velocity.angular = Vector3d::Zero();

        config = {
            .control_loop_hz = 500,
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

        // PIDGains linear_gains = { 0.8, 0.05, 0.15 };
        PIDGains linear_gains = { 1.0, 0.0, 0.0 };
        LinearPID linear_pid(config, linear_gains);
        // PIDGains angular_gains = { 1.0, 1.15, 0.06 };
        PIDGains angular_gains = { 1.0, 0.0, 0.0 };
        AngularPID angular_pid(config, angular_gains);

        trajectory_controller = std::make_unique<Linear_Controller>(linear_pid, angular_pid, config);
        trajectory_controller->robot = this;
    }

    ~Sim_Quadruped() { shutdown(); };

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
        pose_state.velocity = velocity;
        pose_state.dt = GetFrameTime();
        velocity.linear *= pose_state.dt;
        velocity.angular *= pose_state.dt;
        frames.move_in_local_frame(velocity);
    };

    Pose_State
    read_state() override
    {
        return pose_state;
    };
};

int
main()
{

    Sim_Quadruped robot;

    robot.path.path_looping = true;
    // robot.path.add_waypoints(waypoints);
    robot.sensor_manager.init();
    // robot.frames.init(waypoints);
    robot.path.read_json_latlon("ecef_points.json");
    robot.frames.init(robot.path.path_points_all);

    // robot.frames.global_frame.orientation.rotate(Eigen::AngleAxisd(M_PI / 19, -Vector3d::UnitY()));
    // robot.frames.global_frame.orientation.rotate(Eigen::AngleAxisd(M_PI / 2, -Vector3d::UnitZ()));
    // robot.frames.global_frame.orientation.rotate(Eigen::AngleAxisd(M_PI, Vector3d::UnitY()));
    // robot.frames.global_frame.orientation.rotate(Eigen::AngleAxisd(M_PI / 50, Vector3d::UnitY()));
    /*std::cout << robot.frames.global_frame.orientation.rotation() << std::endl;*/
    /*robot.frames.global_frame.orientation.rotate(Eigen::AngleAxisd(-1,
     * Vector3d::UnitY()));*/
    /*robot.frames.global_frame.orientation.rotate(Eigen::AngleAxisd(-1,
     * Vector3d::UnitY()));*/

    Sim_Display sim = Sim_Display(robot, robot.path.path_points_all);
    robot.start();

    sim.display();

    CloseWindow();
    return 0;
}
