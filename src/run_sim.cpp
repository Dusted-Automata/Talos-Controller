#include "cppmap3d.hh"
#include "linear_controller.hpp"
#include "pid.hpp"
#include "raylib.h"
#include "sim.hpp"
#include "transformations.hpp"

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
        linear_pid.output_max = 2.5;
        linear_pid.output_min = 2.0;
        PIDGains angular_gains = { 5.0, 1.15, 0.06 };
        PIDController angular_pid(angular_gains);
        angular_pid.output_max = 2.0;
        angular_pid.output_min = -2.0;

        trajectory_controller = std::make_unique<Linear_Controller>(linear_pid, angular_pid);
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
    // std::vector<Ecef> waypoints = {
    //     { 4100175.6251356260, 476368.7899695045, 4846344.356704135 },
    //     { 4100209.6729529747, 476361.2681338759, 4846316.478097512 },
    //     { 4100218.5394949187, 476445.5598077707, 4846300.796185957 },
    //     { 4100241.7219579100, 476441.0557096391, 4846281.753675706 }
    // };
    //
    std::vector<Ecef> waypoints = {
        { 4100157.662065, 476378.631671, 4846296.665580 },
        { 4100148.690049, 476372.016755, 4846301.445640 },
        { 4100149.702068, 476374.624433, 4846305.056090 },
        { 4100149.701858, 476374.962868, 4846304.499274 },
        { 4100158.341835, 476373.682093, 4846300.376217 },
        { 4100164.617288, 476372.803512, 4846292.699499 },
        { 4100166.832613, 476369.785077, 4846290.870528 },
        { 4100164.228392, 476367.548448, 4846291.181639 }
    };

    std::vector<NED> ned_points = {
        {      0.0,      0.0,       0.0 },
        { -5.53526,   10.474,   2.60001 },
        { -3.06181,  11.8087, -0.999988 },
        { -2.72561,  11.4194, -0.599989 },
        { -4.99496,  2.31725,      -2.9 },
        { -6.59191, -7.32251, -0.999992 },
        { -9.84585, -9.91782, -0.799985 },
        {  -11.767, -7.54495,  0.800016 }
    };

    Ecef start = { 4100157.662065, 476378.631671, 4846296.665580 };
    LLH origin = cppmap3d::ecef2geodetic(start);
    // for (auto point : waypoints) {
    for (auto point : ned_points) {
        // Vector3d ned = wgsecef2ned_d(start, point); // THESE ARE SWITCHED
        // NED ned = cppmap3d::ecef2ned(point, origin); // THESE ARE SWITCHED
        Ecef ned = cppmap3d::ned2ecef(point, origin); // THESE ARE SWITCHED
        std::cout << std::fixed << ned.raw().transpose() << std::endl;
    }

    Sim_Quadruped robot;

    robot.path.path_looping = true;
    robot.path.add_waypoints(waypoints);
    robot.sensor_manager.init();
    robot.frames.init(waypoints);

    // robot.frames.global_frame.orientation.rotate(Eigen::AngleAxisd(M_PI / 19, -Vector3d::UnitY()));
    // robot.frames.global_frame.orientation.rotate(Eigen::AngleAxisd(M_PI / 2, -Vector3d::UnitZ()));
    // robot.frames.global_frame.orientation.rotate(Eigen::AngleAxisd(M_PI, Vector3d::UnitY()));
    // robot.frames.global_frame.orientation.rotate(Eigen::AngleAxisd(M_PI / 50, Vector3d::UnitY()));
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
