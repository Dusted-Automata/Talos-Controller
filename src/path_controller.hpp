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
    std::atomic_bool running{ false };

  public:
    Path_Controller() = default;
    ~Path_Controller() { stop(); }

    bool path_looping = false;
    Thread_Safe_Queue<Ecef_Coord> path_queue;
    std::vector<Ecef_Coord> path_points_all;
    std::thread path_loop_thread;
    std::thread tcp_thread;

    void path_loop();
    void start();
    void stop();

    void add_waypoints(std::vector<Ecef_Coord> &waypoints);
    std::optional<std::pair<Ecef_Coord, Ecef_Coord>> front_two();
    void goal_reached();
};
