#pragma once
#include <Eigen/Dense>
#include <array>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <span>

using Eigen::Affine3d;
using Eigen::Matrix4d;
using Eigen::Vector3d;
typedef Vector3d Ecef_Coord;
typedef Vector3d LLH;
typedef Vector3d Linear_Velocity;
typedef Vector3d Angular_Velocity;

struct Robot_State {
    std::array<float, 3> position; // (unit: m), from own odometry in inertial frame, usually drift
    std::array<float, 3> velocity; // (unit: m/s), forwardSpeed, sideSpeed,
                                   // rotateSpeed in body frame
    float yawSpeed;                // (unit: rad/s), rotateSpeed in body frame
                                   // std::array<MotorState, 20> motorState;
                                   // IMU imu;
};

// Similar to Trajectory_Point

// struct Angular_Velocity {
//   double roll, pitch, yaw;
// };
// struct Linear_Velocity {
//   double forward, lateral, vertical;
// };
struct Velocity2d {
    // high-level representation
    // can have 12 joint angles/torques (3 per leg)
    Linear_Velocity linear;   // x, y, z velocity (m/s)
    Angular_Velocity angular; // roll, pitch, yaw rates (rad/s)
};

struct Pose_State {
    Ecef_Coord position; // x, y, z
    Affine3d orientation;
    // Eigen::Quaterniond orientation; // quaternion
    Velocity2d velocity; // vx, vy, vz \  wx, wy, wz
    double dt;
};

struct Pose {
    Ecef_Coord point;
    Affine3d transformation_matrix; // Change this to Quaternion maybe
};

struct Motion_Constraints {
    double max_velocity;
    double max_acceleration;
    double max_deceleration;
    double max_jerk;
};

struct Robot_Config {
    int hz;
    Motion_Constraints motion_constraints;
};

struct Motion_Step {
    Ecef_Coord &current;
    Ecef_Coord &next;
    Ecef_Coord &difference;
    Affine3d &robot_frame;
    double &dt;
};

struct Trajectory_Point {
    Pose pose;
    double dt;
    Velocity2d velocity;
};

template<typename T, std::size_t Capacity> class Ring_Buffer
{
  private:
    std::array<T, Capacity> buffer = {};
    size_t head = 0;
    size_t tail = 0;
    size_t num_items = 0;
    bool full = false;

  public:
    T &
    operator[](size_t index)
    {
        if (index >= num_items) {
            throw std::out_of_range("Index out of range");
        }

        size_t actual_index = (tail + index) % buffer.size();
        return buffer[actual_index];
    }

    const T &
    operator[](size_t index) const
    {
        if (index >= num_items) {
            throw std::out_of_range("Index out of range");
        }
        size_t actual_index = (tail + index) % buffer.size();
        return buffer[actual_index];
    }

    bool
    empty() const
    {
        return (!full && (head == tail));
    }

    bool
    isFull() const
    {
        return full;
    }

    size_t
    count() const
    {
        return num_items;
    }

    size_t
    capacity() const
    {
        return buffer.size();
    }

    bool
    push(const T &item)
    {
        if (full) return false;

        buffer[head] = item;
        head = (head + 1) % buffer.size();
        num_items++;
        full = (head == tail);
        return true;
    }

    bool
    pop(T &item)
    {
        if (empty()) return false;

        item = buffer[tail];
        tail = (tail + 1) % buffer.size();
        num_items--;
        full = false;
        return true;
    }

    size_t
    write(std::span<const T> src)
    {
        size_t free_slots = buffer.size() - num_items;
        size_t n = std::min(free_slots, src.size());

        size_t contiguous_space = std::min(n, buffer.size() - head);
        std::copy_n(src.begin(), contiguous_space, buffer.data() + head);

        std::copy_n(src.begin() + contiguous_space, n - contiguous_space, buffer.data());

        num_items += n;
        head = (head + n) % buffer.size();
        return n;
    }

    size_t
    read(std::span<T> dst)
    {
        size_t n = std::min(num_items, dst.size());

        size_t contiguous_elements = std::min(n, buffer.size() - tail);
        std::copy_n(buffer.data() + tail, contiguous_elements, dst.begin());

        std::copy_n(buffer.data(), n - contiguous_elements, dst.begin() + contiguous_elements);

        num_items -= n;
        tail = (tail + n) % buffer.size();
        return n;
    }

    bool
    peek(T &item) const
    {
        if (empty()) return false;
        item = buffer[head];
        return true;
    }

    void
    clear()
    {
        head = tail = 0;
        full = false;
        num_items = 0;
    }

    void
    clear(size_t amount)
    {
        if (amount > num_items) {
            clear();
            return;
        }
        tail = (tail + amount) % buffer.size();
        full = false;
        num_items -= amount;
    }
};

template<typename T> class Thread_Safe_Queue
{
  private:
    std::deque<T> queue;
    std::mutex mutex;
    const size_t max_size;

  public:
    Thread_Safe_Queue(size_t max_size = 10000) : max_size(max_size) {}

    bool
    push(const T &point)
    {
        std::unique_lock<std::mutex> lock(mutex);

        if (queue.size() >= max_size) {
            return false;
        }

        queue.push_back(point);
        lock.unlock();
        return true;
    }

    bool
    push(T &&point)
    {
        std::unique_lock<std::mutex> lock(mutex);

        if (queue.size() >= max_size) {
            return false;
        }

        queue.push_back(std::move(point));
        lock.unlock();
        return true;
    }

    std::optional<T>
    front()
    {
        std::unique_lock<std::mutex> lock(mutex);
        if (queue.empty()) {
            return std::nullopt;
        }
        T point = queue.front();
        lock.unlock();
        return point;
    }

    std::optional<std::pair<T, T>>
    front_two()
    {
        std::unique_lock<std::mutex> lock(mutex);
        if (queue.size() < 2) {
            return std::nullopt;
        }
        return std::make_pair(queue[0], queue[1]);
    }

    bool
    pop()
    {
        std::unique_lock<std::mutex> lock(mutex);
        if (queue.empty()) {
            return false;
        }
        queue.pop_front();
        lock.unlock();
        return true;
    }

    bool
    empty()
    {
        std::unique_lock<std::mutex> lock(mutex);
        return queue.empty();
    }

    size_t
    size()
    {
        std::unique_lock<std::mutex> lock(mutex);
        return queue.size();
    }
};
