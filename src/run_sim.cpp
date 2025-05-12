#include "raylib.h"
#include "sim.hpp"

class Sim_Quadruped : public Robot
{

  private:
    bool running = false;
    std::thread control_loop_thread;
    int HZ = 500;

  public:
    Sim_Quadruped()
    { // horizon_steps, num_samples, dt, temperature

        // Initialize robot state
        pose_state.position = Vector3d(0, 0, 0.5); // Starting position with z=0.5 (standing)
        pose_state.orientation = Eigen::Affine3d::Identity();
        pose_state.velocity.linear = Vector3d::Zero();
        pose_state.velocity.angular = Vector3d::Zero();

        // Robot_Config config = {
        //     .hz = 50,
        //     .motion_constraints =
        //         {
        //             .max_velocity = 2.0,
        //             .max_acceleration = 0.5,
        //             .max_deceleration = 0.5,
        //             .max_jerk = 0.0,
        //         },
        // };

        PIDGains linear_gains = { 0.8, 0.05, 0.15 };
        PIDController linear_pid(linear_gains);
        linear_pid.output_max = 10.0;
        linear_pid.output_min = 0.0;
        PIDGains angular_gains = { 1.0, 0.01, 0.25 };
        PIDController angular_pid(angular_gains);
        angular_pid.output_max = 2.0;
        angular_pid.output_min = -2.0;

        trajectory_controller = std::make_unique<Linear_Controller>(linear_pid, angular_pid);
        trajectory_controller->robot = this;
    }

    ~Sim_Quadruped() { shutdown(); };

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
    void
    send_velocity_command(Velocity2d &velocity) override
    {
        pose_state.velocity = velocity;
        pose_state.dt = GetFrameTime();
        velocity.linear *= pose_state.dt;
        velocity.angular *= pose_state.dt;
        frames.move_in_local_frame(velocity);
    };
    void
    start()
    {
        if (running) return;
        running = true;

        if (control_loop_thread.joinable()) {
            control_loop_thread.join();
        }

        control_loop_thread = std::thread(&Sim_Quadruped::control_loop, this);
    }

    void
    shutdown()
    {
        running = false;
        if (control_loop_thread.joinable()) {
            control_loop_thread.join();
        }
    }

    Pose_State
    read_state() override
    {
        return pose_state;
    };

    void
    control_loop()
    {

        while (running) {
            Robot::control_loop();
            std::this_thread::sleep_for(std::chrono::milliseconds(1000 / HZ));
        }
    }
};

int
main()
{
    std::vector<Ecef_Coord> waypoints = {
        { 4100175.6251356260, 476368.7899695045, 4846344.356704135 },
        { 4100209.6729529747, 476361.2681338759, 4846316.478097512 },
        { 4100218.5394949187, 476445.5598077707, 4846300.796185957 },
        { 4100241.7219579100, 476441.0557096391, 4846281.753675706 }
    };

    Sim_Quadruped robot;

    robot.path.path_looping = true;
    robot.path.add_waypoints(waypoints);
    robot.sensor_manager.init();
    // robot.frames.init(robot.path_controller.path_queue.front());
    robot.frames.init(robot.path.path_queue.front_two());

    robot.frames.global_frame.orientation.rotate(Eigen::AngleAxisd(M_PI / 19, -Vector3d::UnitY()));
    robot.frames.global_frame.orientation.rotate(Eigen::AngleAxisd(M_PI / 2, -Vector3d::UnitZ()));
    robot.frames.global_frame.orientation.rotate(Eigen::AngleAxisd(M_PI, Vector3d::UnitY()));
    robot.frames.global_frame.orientation.rotate(Eigen::AngleAxisd(M_PI / 50, Vector3d::UnitY()));
    /*std::cout << robot.frames.global_frame.orientation.rotation() << std::endl;*/
    /*robot.frames.global_frame.orientation.rotate(Eigen::AngleAxisd(-1,
     * Vector3d::UnitY()));*/
    /*robot.frames.global_frame.orientation.rotate(Eigen::AngleAxisd(-1,
     * Vector3d::UnitY()));*/

    Sim_Display sim = Sim_Display(robot, waypoints);
    robot.start();

    sim.display();

    CloseWindow();
    return 0;
}
