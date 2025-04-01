#include "mppi.hpp"
#include "controller.hpp"
#include "types.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

void MPPI_Controller::update(Pose_State &current_state)
{
    // NOTE might pull this into the class and just reuse it. Probably a performance upgrade.
    // std::vector<Trajectory> perturbed_trajectories(num_samples);

    for (int i = 0; i < num_samples; i++)
    {
        perturbed_trajectories[i].controls = generatePerturbedTrajectory();
        perturbed_trajectories[i].cost =
            evaluateTrajectory(current_state, perturbed_trajectories[i]);
    }
    computeWeights(perturbed_trajectories);
    updateNominalTrajectory(perturbed_trajectories);
}

void MPPI_Controller::shiftControlHorizon()
{
    for (int i = 0; i < horizon_steps - 1; ++i)
    {
        nominal_controls.velocities[i] = nominal_controls.velocities[i + 1];
    }

    if (horizon_steps > 1)
    {
        nominal_controls.velocities[horizon_steps - 1] =
            nominal_controls.velocities[horizon_steps - 2];
    }
}

Control_Sequence MPPI_Controller::generatePerturbedTrajectory()
{
    Control_Sequence controls = nominal_controls;

    for (Velocity2d &control : controls)
    {
        // Add noise to each control dimension
        control.linear.x() += 0.5 * dist(gen);
        // control.linear.y() += 0.5 * dist(gen);
        // control.linear.z() += 0.5 * dist(gen);
        // control.angular.x() += 0.3 * dist(gen);
        // control.angular.y() += 0.3 * dist(gen);
        control.angular.z() += 0.3 * dist(gen);
    }

    return controls;
}

double MPPI_Controller::evaluateTrajectory(Pose_State &initial_state, Trajectory &trajectory)
{
    double total_cost = 0.0;
    Pose_State state = initial_state;

    for (int i = 0; i < horizon_steps; ++i)
    {
        state = model.simulate(state, trajectory.controls.velocities[i], dt);

        Ecef_Coord difference = target_position - state.position;
        double azimuth_rad_world = atan2(difference.y(), difference.x());
        double azimuth_rad_robot = atan2(state.orientation(1, 0), state.orientation(0, 0));
        double azimuth_rad = azimuth_rad_world - azimuth_rad_robot;
        if (azimuth_rad > M_PI)
            azimuth_rad -= 2 * M_PI;
        if (azimuth_rad < -M_PI)
            azimuth_rad += 2 * M_PI;

        double orientation_cost = azimuth_rad * 0.3;
        double position_cost = (state.position - target_position).norm();
        double velocity_cost = state.velocity.linear.norm() * 0.1;
        double angular_vel_cost = state.velocity.angular.norm() * 0.2;
        double control_cost = trajectory.controls.velocities[i].linear.norm() * 0.05 +
                              trajectory.controls.velocities[i].angular.norm() * 0.3;

        // Increase cost weight for later time steps to prioritize reaching the goal
        // double time_weight = 1.0 + 0.1 * i;
        double time_weight = 1.0;

        total_cost += time_weight * (position_cost + orientation_cost + velocity_cost +
                                     angular_vel_cost + control_cost);
    }

    return total_cost;
}

void MPPI_Controller::computeWeights(std::vector<Trajectory> &trajectories)
{
    double min_cost = trajectories[0].cost;
    for (Trajectory &trajectory : trajectories)
    {
        if (trajectory.cost < min_cost)
            min_cost = trajectory.cost;
    }

    double sum_weights = 0.0;
    for (int i = 0; i < num_samples; ++i)
    {
        trajectories[i].weight = std::exp(-(trajectories[i].cost - min_cost) / temperature);
        sum_weights += trajectories[i].weight;
    }

    if (sum_weights > 0.0)
    {
        for (int i = 0; i < num_samples; ++i)
        {
            trajectories[i].weight /= sum_weights;
        }
    }
    else
    {
        for (int i = 0; i < num_samples; ++i)
        {
            trajectories[i].weight = 1.0 / num_samples;
        }
    }
}

void MPPI_Controller::updateNominalTrajectory(const std::vector<Trajectory> &trajectories)
{
    for (Velocity2d &control : nominal_controls)
    {
        control.linear.setZero();
        control.angular.setZero();
    }

    for (const Trajectory &trajectory : trajectories)
    {
        for (int i = 0; i < horizon_steps; ++i)
        {
            nominal_controls.velocities[i].linear +=
                trajectory.weight * trajectory.controls.velocities[i].linear;
            nominal_controls.velocities[i].angular +=
                trajectory.weight * trajectory.controls.velocities[i].angular;
        }
    }
}

Velocity2d MPPI_Controller::get_cmd(Pose_State &state, Thread_Safe_Queue<Ecef_Coord> &path_queue)
{
    update(state);
    Velocity2d cmd = nominal_controls.velocities[0];
    shiftControlHorizon();
    return cmd;
}

class QuadrupedRobot
{
  public:
    QuadrupedRobot() : mppi_(50, 1000, 0.02, 0.5)
    { // horizon_steps, num_samples, dt, temperature

        state_.position = Vector3d(0, 0, 0.5); // Starting position with z=0.5 (standing)
        state_.orientation = Eigen::Affine3d::Identity();
        state_.velocity.linear = Vector3d::Zero();
        state_.velocity.angular = Vector3d::Zero();
    }

    void setTargetPosition(const Ecef_Coord &target)
    {
        std::cout << "Setting new target: " << target.transpose() << std::endl;
        mppi_.setTarget(target);
    }

    void applyDisturbance(const Eigen::Vector3d &force, const Eigen::Vector3d &torque)
    {
        std::cout << "Applying disturbance: force=" << force.transpose()
                  << ", torque=" << torque.transpose() << std::endl;

        state_.velocity.linear += force * 0.1;
        state_.velocity.angular += torque * 0.1;
    }

    // Update loop - should be called at regular intervals
    // void update()
    // {
    //     // Get control input from MPPI
    //     Velocity2d control = mppi_.update(state_);
    //
    //     // Apply control to robot
    //     QuadrupedModel model;
    //     state_ = model.simulate(state_, control, 0.02); // 50Hz update rate
    //
    //     // Prepare MPPI for next iteration
    //     mppi_.shiftControlHorizon();
    //
    //     // Print current state
    //     std::cout << "Position: " << state_.position.transpose()
    //               << ", Velocity: " << state_.velocity.linear.transpose() << std::endl;
    // }

  private:
    Pose_State state_;
    MPPI_Controller mppi_;
};

// Example main function
// int main()
// {
//     QuadrupedRobot robot;
//
//     // Set a target 2 meters forward, 1 meter to the right
//     robot.setTargetPosition(Eigen::Vector3d(2.0, 1.0, 0.5));
//
//     // Simulation loop
//     for (int i = 0; i < 200; ++i)
//     { // 4 seconds at 50Hz
//         robot.update();
//
//         // Apply a disturbance halfway through
//         if (i == 100)
//         {
//             robot.applyDisturbance(Eigen::Vector3d(-1.0, 0.5, 0.0), Eigen::Vector3d(0.1, 0.0,
//             0.2));
//         }
//
//         // Simple timing - not suitable for real-time control
//         std::this_thread::sleep_for(std::chrono::milliseconds(20));
//     }
//
//     return 0;
// }
