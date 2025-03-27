#pragma once
#include "types.hpp"
#include <deque>
#include <mutex>
#include <optional>

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

class Controller
{
  public:
    virtual Velocity2d get_cmd(Pose_State &state, Thread_Safe_Queue<Ecef_Coord> &path_queue) = 0;
    bool path_looping = false;
};
