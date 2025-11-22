#include "types.hpp"
#include <arpa/inet.h>
#include "sim_bot.hpp"

void
apply_disturbance(Robot& robot, const Eigen::Vector3d &force, const Eigen::Vector3d &torque)
{
    std::cout << "Applying disturbance: force=" << force.transpose() << ", torque=" << torque.transpose()
        << std::endl;

    robot.pva.linear.velocity += force * 0.1;
    robot.pva.angular.velocity += torque * 0.1;
}


    //
    // sensor.fd = socket(AF_INET, SOCK_DGRAM, 0);
    // if (sensor.fd  < 0) {
    //     perror("socket creation failed!");
    //     return;
    // }
    //
    // // sensor.local_addr.sin_family = AF_INET;
    // // sensor.local_addr.sin_addr.s_addr = INADDR_ANY;  
    // // sensor.local_addr.sin_port = htons(local_port);
    // //
    // // if (bind(sensor.fd, (sockaddr*)&sensor.local_addr, sizeof(sensor.local_addr)) < 0) {
    // //     perror("bind");
    // //     close(sensor.fd);
    // // }
    //
    // sensor.server_addr.sin_family = AF_INET;
    // sensor.server_addr.sin_port = htons(target_port);
    // sensor.server_addr.sin_addr.s_addr = inet_addr(target_IP);
    //
    // std::string msg = "Connect";
    // socklen_t addr_len = sizeof(sensor.server_addr);
    //
    // sendto(sensor.fd, msg.c_str(), msg.length(), 0, (sockaddr*)&sensor.server_addr, addr_len);


typedef struct {
    Vector3f velocity;
    Vector3f acceleration;
} LinearKinematics;

typedef struct {
    Vector3f velocity;
    Vector3f acceleration;
} AngularKinematics;


typedef struct {
    LinearKinematics linear;
    AngularKinematics angular;
} SpatialKinematics;


void
sim_send_velocity_command(void* ctx, Velocity2d &velocity)
{
    Sim_Bot* sim = (Sim_Bot*)ctx;
    // sim_pose.dt = GetFrameTime(); // TODO: Figure out a way to get frame time
    // sim->sim_velocity.linear.velocity = velocity.linear_vel;
    // sim->sim_velocity.angular.velocity = velocity.angular_vel;
    // velocity.linear_vel *= sim_pose.dt;
    // velocity.angular_vel *= sim_pose.dt;

    // velocity.linear_vel *= sim->dt;
    // velocity.angular_vel *= sim->dt;

    // SpatialKinematics kinematics;
// ... fill in data ...

    // sendto(sockfd, &kinematics, sizeof(SpatialKinematics), 0, 
    //        (struct sockaddr*)&server_addr, sizeof(server_addr));
    LinearKinematics lk = {.velocity = velocity.linear_vel.cast<float>(), .acceleration = {} };
    AngularKinematics ak = {.velocity = velocity.angular_vel.cast<float>(), .acceleration = {} };
    SpatialKinematics vel = {.linear = lk, .angular = ak};
    sendto(sim->fd, &vel, sizeof(vel), 0, (sockaddr*)&sim->server_addr, sizeof(sim->server_addr));
};


void
sim_init(void* ctx, const Robot* robot)
{
    std::cout << "in sim_init" << std::endl;
    Sim_Bot* sim = (Sim_Bot*)ctx;
    // sim->turn_right_constraint = robot->config.kinematic_constraints.velocity_turning_right_max;
    // sim->turn_left_constraint = robot->config.kinematic_constraints.velocity_turning_left_max;
    sim->fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sim->fd  < 0) {
        perror("socket creation failed!");
        return;
    }

    sim->server_addr.sin_family = AF_INET;
    sim->server_addr.sin_port = htons(9999);
    sim->server_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
}

