#pragma once
#include <functional>
#include <thread>

inline void worker_function(std::function<void()> callback, int period_ms)
{
    while (1)
    {
        callback();
        std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
    }
}

class Path_Controller
{

    // std::function<void()> bound_path_loop = std::bind(
    //     &Linear_Controller::path_loop, &l_c, std::ref(robot.path_queue), std::ref(waypoints));
    // std::thread path_loop = std::thread(worker_function, bound_path_loop, 30);
};
