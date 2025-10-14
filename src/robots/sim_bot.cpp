#include "frames.hpp"
#include "linear_controller.hpp"
#include "load_config.hpp"
#include "path_planner.hpp"
#include "pid.hpp"
#include "transformations.hpp"
#include "sim.hpp"

struct Config : public Robot_Config {
    PIDGains linear_gains;
    PIDGains angular_gains;
};


double inertia = 0.02;
double damping = 0.05;    
double angular_velocity = 0.0;    // ω
double dt = 0.01;
double turn_left_constraint = 0.1;
double turn_right_constraint = -0.1;

LA sim_velocity = {};

class Sim_Bot : public Robot
{


  public:
    Sim_Bot()
    {
        pva.pose.local_point = Eigen::Vector3d(0, 0, 0.5);
        pva.pose.transformation_matrix = Eigen::Affine3d::Identity();
        pva.linear.velocity = Vector3d::Zero();
        pva.linear.acceleration = Vector3d::Zero();
        pva.angular.velocity = Vector3d::Zero();
        pva.angular.acceleration = Vector3d::Zero();
    }

    void
    applyDisturbance(const Eigen::Vector3d &force, const Eigen::Vector3d &torque)
    {
        std::cout << "Applying disturbance: force=" << force.transpose() << ", torque=" << torque.transpose()
                  << std::endl;

        pva.linear.velocity += force * 0.1;
        pva.angular.velocity += torque * 0.1;
    }
    void
    send_velocity_command(Velocity2d &velocity) override
    {
        // sim_pose.dt = GetFrameTime(); // TODO: Figure out a way to get frame time
        sim_velocity.linear.velocity = velocity.linear_vel;
        sim_velocity.angular.velocity = velocity.angular_vel;
        // velocity.linear_vel *= sim_pose.dt;
        // velocity.angular_vel *= sim_pose.dt;

        velocity.linear_vel *= dt;
        velocity.angular_vel *= dt;
    };

};

LA
read_state() 
{
    // sim_pose.dt = GetFrameTime(); // TODO: Figure out a way to get frame time
    double angular_acceleration = (sim_velocity.angular.velocity.z() - damping * angular_velocity) / inertia;
    angular_velocity += angular_acceleration * dt;
    // angular_velocity = std::clamp(angular_velocity, config.kinematic_constraints.velocity_turning_right_max, config.kinematic_constraints.velocity_turning_left_max);
    angular_velocity = std::clamp(angular_velocity, turn_right_constraint, turn_left_constraint );
    sim_velocity.angular.velocity.z()  = angular_velocity;

    return sim_velocity;
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

    // robot.server.init(robot);
    server_init(robot.server, robot);
    robot.paused = false;
    robot.running = true;
}

int
main()
{

    Sim_Bot robot;
    load_config(robot, "robot_configs/sim_bot.json");
    turn_right_constraint = robot.config.kinematic_constraints.velocity_turning_right_max;
    turn_left_constraint = robot.config.kinematic_constraints.velocity_turning_left_max;
    Path path = read_json_latlon(robot.config.path_config.filepath); // NEW
    Path_Cursor p_cursor; // NEW
    p_cursor.initialize(&path);
    robot.path.path_direction = robot.config.path_config.direction;
    robot.path.global_path.read_json_latlon(robot.config.path_config.filepath);
    robot.path.gen_local_path(robot.config.path_config.interpolation_distances_in_meters);

    robot.path.global_cursor = &p_cursor; // NEW

    robot.read_pv = *read_state;

    { // Find out how to extract this.

        Trapezoidal_Profile linear_profile(robot.config.kinematic_constraints.velocity_forward_max,
            robot.config.kinematic_constraints.acceleration_max,
            robot.config.kinematic_constraints.velocity_backward_max,
            robot.config.kinematic_constraints.deceleration_max);
        Linear_Controller traj_controller(robot.config.linear_gains, robot.config.angular_gains, linear_profile);
        frames_init(robot.frames, robot.path.local_path);
        init_bot(robot);
        std::thread control_thread(control_loop<Sim_Bot>, std::ref(robot), std::ref(traj_controller));

        // control_loop<Sim_Bot>(robot, traj_controller);

        Sim_Display sim = Sim_Display(robot, robot.path);
        sim.display();
        robot.running = false;
    }

    return 0;
}
