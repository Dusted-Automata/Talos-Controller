#pragma once
#include "trajectory.hpp"
#include "types.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <random>
#include <vector>

class QuadrupedModel
{
  public:
    Pose_State simulate(Pose_State &current_state, Velocity2d &control, double dt)
    {
        // Simplified dynamics model (you'd replace this with your robot's actual dynamics)
        Pose_State next_state = current_state;

        // Update position based on velocity
        next_state.position += current_state.velocity.linear * dt;

        // Update velocity with some damping and the control input
        const double damping = 0.9;
        next_state.velocity.linear =
            damping * current_state.velocity.linear + (1.0 - damping) * control.linear;

        // Simple quaternion integration for orientation
        // Eigen::Quaterniond q = current_state.orientation;
        // Eigen::Vector3d w = current_state.angular_vel;
        //
        // // Quaternion derivative (simplified)
        // Eigen::Quaterniond q_dot;
        // q_dot.x() = -0.5 * (q.y() * w(0) + q.z() * w(1) + q.w() * w(2));
        // q_dot.y() = 0.5 * (q.x() * w(0) + q.z() * w(2) - q.w() * w(1));
        // q_dot.z() = 0.5 * (q.x() * w(1) + q.w() * w(0) - q.y() * w(2));
        // q_dot.w() = 0.5 * (q.x() * w(2) + q.y() * w(1) - q.z() * w(0));

        // next_state.orientation += q_dot * dt;
        // Normalize quaternion

        // Update angular velocity with damping and the control input
        next_state.velocity.angular =
            damping * current_state.velocity.angular + (1.0 - damping) * control.angular;

        return next_state;
    }
};

struct Control_Sequence
{
    std::vector<Velocity2d> velocities;

    auto begin() { return velocities.begin(); }
    auto end() { return velocities.end(); }
    auto begin() const { return velocities.begin(); }
    auto end() const { return velocities.end(); }
};

struct Trajectory
{
    Control_Sequence controls;
    double cost;
    double weight;
};

class MPPI_Controller : public Controller
{
  private:
    int horizon_steps;
    int num_samples;
    double temperature;
    Ecef_Coord target_position;
    QuadrupedModel model;

    std::mt19937 gen;
    std::normal_distribution<double> dist;

    // Generate a perturbed trajectory by adding noise to the nominal trajectory
    Control_Sequence generatePerturbedTrajectory();
    // Evaluate a trajectory by simulating and computing cost
    double evaluateTrajectory(Pose_State &initial_state, Trajectory &controls);
    // Compute importance sampling weights based on trajectory costs
    void computeWeights(std::vector<Trajectory> &trajectories);
    // Update nominal trajectory based on weighted average of perturbed trajectories
    void updateNominalTrajectory(const std::vector<Trajectory> &trajectories);

  public:
    MPPI_Controller(int horizon_steps, int num_samples, double dt, double temperature)
        : horizon_steps(horizon_steps), num_samples(num_samples), dt(dt), temperature(temperature),
          perturbed_trajectories(num_samples)
    {

        // Initialize random number generator
        std::random_device rd;
        gen = std::mt19937(rd());
        dist = std::normal_distribution<double>(0.0, 1.0);

        // surely there is some better way to zero init a vector? like what the hell?
        nominal_controls.velocities.resize(horizon_steps);
        for (auto &control : nominal_controls)
        {
            control.linear = Vector3d::Zero();
            control.angular = Vector3d::Zero();
        }
    }

    void update(Pose_State &current_state);
    void setTarget(const Ecef_Coord &target) { target_position = target; }
    Velocity2d get_cmd(Pose_State &state, Thread_Safe_Queue<Ecef_Coord> &path_queue) override;
    void shiftControlHorizon();
    double dt;
    std::vector<Trajectory> perturbed_trajectories;
    Control_Sequence nominal_controls;
};
