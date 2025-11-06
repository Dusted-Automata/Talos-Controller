#include "robot.hpp"

bool
Robot::pause()
{
    paused = true;
    return true;
}

bool
Robot::stop()
{
    running = false;
    return true;
}

bool
Robot::resume()
{
    paused = false;
    return true;
}

PVA
Robot::get_PVA()
{
    return pva;
}

void
Robot::init(void (*init_ctx)(void*, const Robot* robot))
{
    pva.pose.local_point = Eigen::Vector3d(0, 0, 0);
    pva.pose.transformation_matrix = Eigen::Affine3d::Identity();
    pva.linear.velocity = Vector3d::Zero();
    pva.linear.acceleration = Vector3d::Zero();
    pva.angular.velocity = Vector3d::Zero();
    pva.angular.acceleration = Vector3d::Zero();

    init_ctx(ctx, this);
    paused = false;
    running = true;
}
