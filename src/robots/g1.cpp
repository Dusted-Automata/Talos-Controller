#include "g1.hpp"
#include "sim.hpp"
#include <chrono>
#include <iostream>
#include <thread>

#include <unitree/robot/g1/loco/g1_loco_api.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>



void
g1_send_velocity_command(void* ctx, Velocity2d &velocity)
{
    G1* g1 = (G1*)ctx;
    // pose_state.velocity = velocity; // FOR DEADRECKONING
    // client.Move(velocity.linear_vel.x(), velocity.linear_vel.y(), velocity.angular_vel.z());
    g1.client.SetVelocity(velocity.linear_vel.x(), velocity.linear_vel.y(), velocity.angular_vel.z(), 0.1f);
};



void
g1_init(void* ctx, const Robot* robot){
    G1* g1 = (G1*)ctx;
    unitree::robot::ChannelFactory::Instance()->Init(0, args["network_interface"]);
}

// void
// go1_deinit(void* ctx){
//     Go1* go1 = (Go1*)ctx;
//
//     go1->loop_udpSend->shutdown();
//     go1->loop_udpRecv->shutdown();
// }


// int main(int argc, char const *argv[]) {
//     G1 robot;
//
//     robot.path.path_direction = robot.config.path_config.direction;
//     robot.path.global_path.read_json_latlon(robot.config.path_config.filepath);
//     robot.path.gen_local_path(robot.config.path_config.interpolation_distances_in_meters);
//
//     Trapezoidal_Profile linear_profile(robot.config.kinematic_constraints.velocity_forward_max,
//         robot.config.kinematic_constraints.acceleration_max, robot.config.kinematic_constraints.velocity_backward_max,
//         robot.config.kinematic_constraints.deceleration_max);
//     Linear_Controller traj_controller(robot.config.linear_gains, robot.config.angular_gains, linear_profile);
//
//     frames_init(robot.frames, robot.path.local_path);
//
//     std::jthread sim_thread(control_loop<G1>, std::ref(robot), std::ref(traj_controller));
//
//     Sim_Display sim = Sim_Display(robot, robot.path);
//     sim.display();
//
//     CloseWindow();
//
//
// }
