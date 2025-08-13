#include "frames.hpp"
#include "linear_controller.hpp"
#include "load_config.hpp"
#include "path_planner.hpp"
#include "pid.hpp"
#include "sim.hpp"
#include "transformations.hpp"

struct Config : public Robot_Config {
    PIDGains linear_gains;
    PIDGains angular_gains;
    Heading heading;
};

class Sim_Bot : public Robot
{
    Pose_State sim_pose = {};

  public:
    Sim_Bot()
    {
        pose_state.position = Vector3d(0, 0, 0.5); // Starting position with z=0.5 (standing)
        pose_state.orientation = Eigen::Affine3d::Identity();
        pose_state.velocity.linear_vel = Vector3d::Zero();
        pose_state.velocity.angular_vel = Vector3d::Zero();
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
};

void
init_bot(Sim_Bot &robot)
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
    //         rotationMatrix = Eigen::AngleAxisd(robot.heading.initial_heading_in_radians,
    //             Eigen::Vector3d::UnitZ());
    //         robot.frames.local_frame.orientation = rotationMatrix; // NOTE: To be checked!
    //         break;
    //     }
    // }

    robot.TCP_reader.init(robot);
    robot.running = true;
}

int
main()
{

    Sim_Bot robot;
    load_config(robot, "robot_configs/sim_bot.json");
    robot.path.path_direction = robot.config.path_config.direction;
    robot.path.global_path.read_json_latlon(robot.config.path_config.filepath);
    robot.path.gen_local_path(robot.config.path_config.interpolation_distances_in_meters);

    { // Find out how to extract this.

        Trapezoidal_Profile linear_profile(robot.config.kinematic_constraints.velocity_forward_max,
            robot.config.kinematic_constraints.acceleration_max,
            robot.config.kinematic_constraints.velocity_backward_max,
            robot.config.kinematic_constraints.deceleration_max);
        Linear_Controller traj_controller(robot.config.linear_gains, robot.config.angular_gains, linear_profile);
        frames_init(robot.frames, robot.path.local_path);
        init_bot(robot);
        std::jthread control_thread(control_loop<Sim_Bot>, std::ref(robot), std::ref(traj_controller));

        // Sim_Display sim = Sim_Display(robot, robot.path);
        Sim_Display sim = Sim_Display(robot, robot.path);
        sim.display();
        robot.running = false;
    }

    return 0;
}
