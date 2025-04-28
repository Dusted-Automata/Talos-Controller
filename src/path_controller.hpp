#pragma once
#include "types.hpp"
#include <functional>
#include <thread>

inline void
worker_function(std::function<void()> callback, double period_seconds)
{
    int period_rounded = static_cast<int>(1000 * period_seconds);
    while (1) {
        callback();
        std::this_thread::sleep_for(std::chrono::milliseconds(period_rounded));
    }
}

class Path_Controller
{
    Thread_Safe_Queue<Ecef_Coord> path_queue;

  public:
    bool path_looping = false;
    std::vector<Ecef_Coord> path_points_all;
    std::thread path_loop_thread;
    std::thread tcp_thread;

    // std::function<void()> bound_path_loop = std::bind(
    //     &Linear_Controller::path_loop, &l_c, std::ref(robot.path_queue), std::ref(waypoints));
    // std::thread path_loop = std::thread(worker_function, bound_path_loop, 30);

    void path_loop();
    void start();

    void add_waypoints(std::vector<Ecef_Coord> &waypoints);
    std::optional<std::pair<Ecef_Coord, Ecef_Coord>> front_two();
    void goal_reached();
};
