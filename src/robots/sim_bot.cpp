#include "frames.hpp"
#include "linear_controller.hpp"
#include "path_planner.hpp"
#include "pid.hpp"
#include "sim.hpp"
#include "transformations.hpp"
#include "ublox.hpp"
#include <chrono>

struct Config : public Robot_Config {
    PIDGains linear_gains;
    PIDGains angular_gains;
    Heading heading;
};

class Sim_Quadruped : public Robot
{
    Pose_State sim_pose = {};

  public:
    Sim_Quadruped()
    {
        pose_state.position = Vector3d(0, 0, 0.5); // Starting position with z=0.5 (standing)
        pose_state.orientation = Eigen::Affine3d::Identity();
        pose_state.velocity.linear_vel = Vector3d::Zero();
        pose_state.velocity.angular_vel = Vector3d::Zero();

        config.control_loop_hz = 15;
        config.goal_tolerance_in_meters = 0.75;
        config.kinematic_constraints = {
            .v_max = 2.5,
            .v_min = 0.0,
            .omega_max = 2.5,
            .omega_min = -2.5,
            .a_max = 1.0,
            .a_min = -1.0,
            .j_max = 0.0,
        };
        config.linear_gains = {
            .k_p = 1.01,
            .k_i = 0.05,
            .k_d = 0.15,
            .output_min = config.kinematic_constraints.v_min,
            .output_max = config.kinematic_constraints.v_max,
            .integral_min = -100,
            .integral_max = 100,
        };

        config.angular_gains = {
            .k_p = 1.0,
            .k_i = 0.01,
            .k_d = 0.25,
            .output_min = config.kinematic_constraints.omega_min,
            .output_max = config.kinematic_constraints.omega_max,
            .integral_min = -100,
            .integral_max = 100,
        };
    }

    void
    applyDisturbance(const Eigen::Vector3d &force, const Eigen::Vector3d &torque)
    {
        std::cout << "Applying disturbance: force=" << force.transpose() << ", torque=" << torque.transpose()
                  << std::endl;

        pose_state.velocity.linear_vel += force * 0.1;
        pose_state.velocity.angular_vel += torque * 0.1;
    }
    void
    send_velocity_command(Velocity2d &velocity) override
    {

        sim_pose.velocity = velocity;
        sim_pose.dt = GetFrameTime();
        velocity.linear_vel *= sim_pose.dt;
        velocity.angular_vel *= sim_pose.dt;
    };

    Pose_State
    read_state() override
    {
        return sim_pose;
    };

    // Ublox ublox = {};
    Config config = {};
};

void
init_bot(Sim_Quadruped &robot)
{
    std::cout << "ROBOT INIT!" << std::endl;
    bool ublox_start = robot.ublox.start();
    std::cout << "UBLOX: " << ublox_start << std::endl;
    // while (true && ublox_start) {
    //     std::optional<Nav_Pvat> msg = robot.ublox.get_latest<Nav_Pvat>(Msg_Type::NAV_PVAT);
    //     if (msg.has_value()) {
    //         std::cout << "VEH_HEADING IS " << msg->veh_heading << std::endl;
    //         std::cout << "MOT_HEADING IS " << msg->mot_heading << std::endl;
    //         robot.heading.initial_heading_in_radians = msg->veh_heading;
    //         Eigen::Matrix3d rotationMatrix;
    //         rotationMatrix = Eigen::AngleAxisd(robot.heading.initial_heading_in_radians - M_PI / 2,
    //             Eigen::Vector3d::UnitZ()); // ENU to NED Correction (-M_PI/2)
    //         robot.frames.local_frame.orientation = rotationMatrix; // NOTE: To be checked!
    //         break;
    //     }
    // }

    robot.running = true;
}

void
control_loop(Sim_Quadruped &robot, Linear_Controller &controller)
{

    using clock = std::chrono::steady_clock;
    auto next = clock::now();
    auto previous_time = clock::now();
    auto motion_time_start = clock::now();
    std::chrono::milliseconds period(1000 / robot.config.control_loop_hz);

    while (robot.running) {               // Control loop
        auto current_time = clock::now(); // Current iteration time
        std::chrono::duration<double> elapsed = current_time - previous_time;
        double dt = elapsed.count(); // `dt` in seconds
        previous_time = current_time;

        if (!robot.pause) {
            robot.pose_state = robot.read_state();
            bool update_speed = robot.ublox.update_speed(robot.pose_state.velocity); // Currently blocking!!
            // std::cout << "ublox_update_speed: " << update_speed << std::endl;
            frames_move_in_local_frame(robot.frames, robot.pose_state.velocity, dt);
            update_position(robot.ublox, robot.frames);
            update_heading(robot.ublox, robot.frames, robot.heading);
            robot.logger.savePosesToFile(robot.frames);
            robot.logger.saveTimesToFile(std::chrono::duration<double>(clock::now() - motion_time_start).count());

            // Pose target_waypoint = robot.path.global_path.next();
            Velocity2d cmd = { .linear_vel = Linear_Velocity().setZero(), .angular_vel = Angular_Velocity().setZero() };

            Vector3d global_dif = frames_diff(robot.frames, robot.path.global_path.next().local_point);
            Vector3d local_dif = frames_diff(robot.frames, robot.path.path.next().local_point);
            if (eucledean_xy_norm(global_dif) > robot.config.goal_tolerance_in_meters) {
                cmd = controller.get_cmd(robot.pose_state, global_dif, local_dif, dt);
                // std::cout << "cmd: " << cmd.linear.transpose() << std::endl;
            } else {
                robot.path.global_path.progress();
            }

            // std::cout << "local_dif: " << local_dif.transpose() << std::endl;
            if (eucledean_xy_norm(local_dif) < robot.config.goal_tolerance_in_meters) {
                controller.motion_profile.reset();
                if (robot.path.path.progress()) {
                    // Vector3d dif = frames_diff(target_waypoint.local_point, robot.frames);
                    controller.motion_profile.set_setpoint(eucledean_xy_norm(local_dif));
                } else {
                    break;
                }
            }
            robot.send_velocity_command(cmd);
        }

        next += period;
        std::this_thread::sleep_until(next);

        if (clock::now() > next + period) {
            std::cerr << "control-loop overrun" << std::endl;
            next = clock::now();
            previous_time = next;
        }
    }
}

int
main()
{

    Sim_Quadruped robot;

    { // Find out how to extract this.

        Trapezoidal_Profile linear_profile(robot.config.kinematic_constraints.v_max,
            robot.config.kinematic_constraints.a_max, robot.config.kinematic_constraints.v_min,
            robot.config.kinematic_constraints.a_min);
        Linear_Controller traj_controller(robot.config.linear_gains, robot.config.angular_gains, linear_profile);

        robot.path.path.path_looping = false;
        // robot.path.path.read_json_latlon("waypoints/_Parkinglot_Loop_short.json");
        robot.path.path.read_json_latlon("waypoints/_Parkinglot_ping_pong.json");
        // robot.path.path.read_json_latlon("waypoints/_Table_Grab_with_corrections.json");
        // robot.path.path.read_json_latlon("waypoints/_basketball_loop.json");
        // robot.path.path.read_json_latlon("waypoints/_ramp_over_parkinglot.json");
        // robot.path.path.read_json_latlon("waypoints/_ramp_over_parkinglot2.json");
        // robot.path.path.read_json_latlon("waypoints/_shotter_weg_loop.json");
        robot.path.gen_global_path(2.5);
        frames_init(robot.frames, robot.path.path);
        init_bot(robot);
        // robot.path.path.print();
        // robot.path.global_path.print();
        // robot.frames.init(robot.path.global_path);
        std::jthread sim_thread(control_loop, std::ref(robot), std::ref(traj_controller));

        // Sim_Display sim = Sim_Display(robot, robot.path);
        Sim_Display sim = Sim_Display(robot, robot.path);
        sim.display();
        robot.running = false;
    }

    return 0;
}
