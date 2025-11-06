#include "transformations.hpp"
#include "sim_bot.hpp"

void
apply_disturbance(Robot& robot, const Eigen::Vector3d &force, const Eigen::Vector3d &torque)
{
    std::cout << "Applying disturbance: force=" << force.transpose() << ", torque=" << torque.transpose()
        << std::endl;

    robot.pva.linear.velocity += force * 0.1;
    robot.pva.angular.velocity += torque * 0.1;
}


void
sim_send_velocity_command(void* ctx, Velocity2d &velocity)
{
    Sim_Bot* sim = (Sim_Bot*)ctx;
    // sim_pose.dt = GetFrameTime(); // TODO: Figure out a way to get frame time
    sim->sim_velocity.linear.velocity = velocity.linear_vel;
    sim->sim_velocity.angular.velocity = velocity.angular_vel;
    // velocity.linear_vel *= sim_pose.dt;
    // velocity.angular_vel *= sim_pose.dt;

    velocity.linear_vel *= sim->dt;
    velocity.angular_vel *= sim->dt;
};

LA
sim_read_state(void* ctx) 
{

    Sim_Bot* sim = (Sim_Bot*)ctx;
    // sim_pose.dt = GetFrameTime(); // TODO: Figure out a way to get frame time
    double angular_acceleration = (sim->sim_velocity.angular.velocity.z() - sim->damping * sim->angular_velocity)
                                  / sim->inertia;
    sim->angular_velocity += angular_acceleration * sim->dt;
    // angular_velocity = std::clamp(angular_velocity, config.kinematic_constraints.velocity_turning_right_max, config.kinematic_constraints.velocity_turning_left_max);
    sim->angular_velocity = std::clamp(sim->angular_velocity, sim->turn_right_constraint, sim->turn_left_constraint );
    sim->sim_velocity.angular.velocity.z()  = sim->angular_velocity;

    return sim->sim_velocity;
};

void
sim_init(void* ctx, const Robot* robot)
{
    std::cout << "in sim_init" << std::endl;
    Sim_Bot* sim = (Sim_Bot*)ctx;
    sim->turn_right_constraint = robot->config.kinematic_constraints.velocity_turning_right_max;
    sim->turn_left_constraint = robot->config.kinematic_constraints.velocity_turning_left_max;
}

