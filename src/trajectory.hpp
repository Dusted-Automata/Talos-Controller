#pragma once
#include "controller.hpp"
#include "pid.hpp"
#include "types.hpp"
#include <deque>
#include <mutex>
#include <optional>
#include <vector>

template <typename T> class Thread_Safe_Queue
{
  private:
    std::deque<T> queue;
    std::mutex mutex;
    const size_t max_size;

  public:
    Thread_Safe_Queue(size_t max_size = 10000) : max_size(max_size) {}

    bool push(const T &point)
    {
        std::unique_lock<std::mutex> lock(mutex);

        if (queue.size() >= max_size)
        {
            return false;
        }

        queue.push_back(point);
        lock.unlock();
        return true;
    }

    bool push(T &&point)
    {
        std::unique_lock<std::mutex> lock(mutex);

        if (queue.size() >= max_size)
        {
            return false;
        }

        queue.push_back(std::move(point));
        lock.unlock();
        return true;
    }

    std::optional<T> front()
    {
        std::unique_lock<std::mutex> lock(mutex);
        if (queue.empty())
        {
            return std::nullopt;
        }
        T point = queue.front();
        lock.unlock();
        return point;
    }

    std::optional<std::pair<T, T>> front_two()
    {
        std::unique_lock<std::mutex> lock(mutex);
        if (queue.size() < 2)
        {
            return std::nullopt;
        }
        return std::make_pair(queue[0], queue[1]);
    }

    bool pop()
    {
        std::unique_lock<std::mutex> lock(mutex);
        if (queue.empty())
        {
            return false;
        }
        queue.pop_front();
        lock.unlock();
        return true;
    }

    bool empty()
    {
        std::unique_lock<std::mutex> lock(mutex);
        return queue.empty();
    }

    size_t size()
    {
        std::unique_lock<std::mutex> lock(mutex);
        return queue.size();
    }
};

class Trajectory_Controller : public Controller
{

  private:
    Robot_Config config;
    PIDController linear_pid;
    PIDController angular_pid;
    double trajectory_time = 0.0;
    double sampling_rate = 1.0;
    bool added_paths = false;

    bool (*sendVelocityCommand)(Linear_Velocity &, Angular_Velocity &);
    // bool (*getState)(Robot_State &);

    Trajectory_Point trajectory_turn(Motion_Step step);
    std::vector<Trajectory_Point> trajectory_ramp_up(Motion_Step step);
    Trajectory_Point trajectory_cruise(Motion_Step step);
    std::vector<Trajectory_Point> trajectory_ramp_down(Motion_Step step);
    Trajectory_Point new_trajectory_point(Affine3d &robot_frame, Ecef_Coord next_point,
                                          Vector3d linear, Vector3d angular, double dt);

  public:
    Trajectory_Controller(Robot_Config config, PIDController linear_pid, PIDController angular_pid,
                          double sampling_rate)
        : config(config), linear_pid(linear_pid), angular_pid(angular_pid),
          sampling_rate(sampling_rate){};

    std::vector<Trajectory_Point> generate_trajectory(Ecef_Coord current, Ecef_Coord next);
    Pose get_current_pose();
    Velocity2d follow_trajectory(Pose_State &state);

    void path_loop(Thread_Safe_Queue<Ecef_Coord> &path, std::vector<Ecef_Coord> &waypoints);
    void trajectory_loop(Thread_Safe_Queue<Ecef_Coord> &waypoints);
    void local_replanning();
    Velocity2d get_cmd(Pose_State &state) override;

    Thread_Safe_Queue<Trajectory_Point> trajectories;
};
