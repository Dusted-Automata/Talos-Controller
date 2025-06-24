#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wextra"
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wclass-memaccess"
#include <Eigen/Dense>
#pragma GCC diagnostic pop
#include <array>
#include <deque>
#include <mutex>
#include <optional>
#include <span>

using Eigen::Affine3d;
using Eigen::Matrix4d;
using Eigen::Vector3d;
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

struct LLH {
  private:
    Eigen::Vector3d v;

  public:
    LLH() {}
    LLH(double lat, double lon, double h) : v(lat, lon, h) {}
    LLH(const Eigen::Vector3d &vec) : v(vec) {}

    double
    lat() const
    {
        return v.x();
    }

    double
    lon() const
    {
        return v.y();
    }

    double
    alt() const
    {
        return v.z();
    }

    // Setter methods (non-const)
    double &
    lat()
    {
        return v.x();
    }

    double &
    lon()
    {
        return v.y();
    }

    double &
    alt()
    {
        return v.z();
    }

    Vector3d
    operator-(const LLH &other) const
    {
        return Vector3d(v - other.v);
    }

    LLH
    operator+(const Vector3d &other) const
    {
        return LLH(v + other);
    }

    LLH
    operator-(const Vector3d &other) const
    {
        return LLH(v - other);
    }

    LLH &
    operator+=(const Vector3d &other)
    {
        v += other;
        return *this;
    }

    LLH &
    operator-=(const Vector3d &other)
    {
        v -= other;
        return *this;
    }

    const Eigen::Vector3d &
    raw() const
    {
        return v;
    }
};

struct Ecef {
  private:
    Eigen::Vector3d v;

  public:
    Ecef() {}
    Ecef(double x, double y, double z) : v(x, y, z) {}
    Ecef(const Eigen::Vector3d &vec) : v(vec) {}

    double
    x() const
    {
        return v.x();
    }
    double
    y() const
    {
        return v.y();
    }
    double
    z() const
    {
        return v.z();
    }

    double &
    x()
    {
        return v.x();
    }
    double &
    y()
    {
        return v.y();
    }
    double &
    z()
    {
        return v.z();
    }

    Vector3d
    operator-(const Ecef &other) const
    {
        return Vector3d(v - other.v);
    }

    Ecef
    operator+(const Vector3d &other) const
    {
        return Ecef(v + other);
    }

    Ecef
    operator-(const Vector3d &other) const
    {
        return Ecef(v - other);
    }

    Ecef &
    operator+=(const Vector3d &other)
    {
        v += other;
        return *this;
    }

    Ecef &
    operator-=(const Vector3d &other)
    {
        v -= other;
        return *this;
    }

    const Eigen::Vector3d &
    raw() const
    {
        return v;
    }
};

struct ENU {
  private:
    Eigen::Vector3d v;

  public:
    ENU() {}
    ENU(double lat, double lon, double h) : v(lat, lon, h) {}
    ENU(const Eigen::Vector3d &vec) : v(vec) {}

    double
    east() const
    {
        return v.x();
    }

    double
    north() const
    {
        return v.y();
    }

    double
    up() const
    {
        return v.z();
    }

    // Setter methods (non-const)
    double &
    north()
    {
        return v.x();
    }

    double &
    east()
    {
        return v.y();
    }

    double &
    down()
    {
        return v.z();
    }

    Vector3d
    operator-(const ENU &other) const
    {
        return Vector3d(v - other.v);
    }

    ENU
    operator+(const Vector3d &other) const
    {
        return ENU(v + other);
    }

    ENU
    operator-(const Vector3d &other) const
    {
        return ENU(v - other);
    }

    ENU &
    operator+=(const Vector3d &other)
    {
        v += other;
        return *this;
    }

    ENU &
    operator-=(const Vector3d &other)
    {
        v -= other;
        return *this;
    }

    const Eigen::Vector3d &
    raw() const
    {
        return v;
    }
};

struct NED {
  private:
    Eigen::Vector3d v;

  public:
    NED() {}
    NED(double lat, double lon, double h) : v(lat, lon, h) {}
    NED(const Eigen::Vector3d &vec) : v(vec) {}

    double
    north() const
    {
        return v.x();
    }

    double
    east() const
    {
        return v.y();
    }

    double
    down() const
    {
        return v.z();
    }

    // Setter methods (non-const)
    double &
    north()
    {
        return v.x();
    }

    double &
    east()
    {
        return v.y();
    }

    double &
    down()
    {
        return v.z();
    }

    Vector3d
    operator-(const NED &other) const
    {
        return Vector3d(v - other.v);
    }

    NED
    operator+(const Vector3d &other) const
    {
        return NED(v + other);
    }

    NED
    operator-(const Vector3d &other) const
    {
        return NED(v - other);
    }

    NED &
    operator+=(const Vector3d &other)
    {
        v += other;
        return *this;
    }

    NED &
    operator-=(const Vector3d &other)
    {
        v -= other;
        return *this;
    }

    const Eigen::Vector3d &
    raw() const
    {
        return v;
    }
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
    Linear_Velocity linear;   // x, y, z velocity (m/s)
    Angular_Velocity angular; // roll, pitch, yaw rates (rad/s)
};

static inline double
to_radian(double degrees)
{
    double rad = degrees * (M_PI / 180);
    const double TWO_PI = 2.0 * M_PI;
    rad = fmod(rad + M_PI, TWO_PI);
    if (rad < 0) rad += TWO_PI;
    return rad - M_PI;
}

struct Pose_State {
    Ecef position = Eigen::Vector3d(0, 0, 0); // x, y, z
    Affine3d orientation = Eigen::Affine3d::Identity();
    // Eigen::Quaterniond orientation; // quaternion
    Velocity2d velocity = { .linear = Vector3d::Zero(), .angular = Vector3d::Zero() }; // vx, vy, vz \  wx, wy, wz
    double dt;
};

struct Pose {
    Ecef point;
    Affine3d transformation_matrix; // Change this to Quaternion maybe
};

struct Kinematic_Constraints {
    double v_max = 0;       // max linear velocity (m/s)
    double v_min = 0;       // min linear velocity (m/s)
    double omega_max = 0;   // max angular velocity (rad/s)
    double omega_min = 0;   // min angular velocity (rad/s)
    double a_max = 0;       // max linear acceleration (m/s^2)
    double a_min = 0;       // max linear deceleration (m/s^2), possibly negative
    double alpha_max = 0;   // max angular acceleration
    double j_max = 0;       // max Linear jerk (m/s^3)
    double j_omega_max = 0; // max Angular jerk (m/s^3)
};

struct Robot_Config {
    int control_loop_hz;
    double goal_tolerance_in_meters;
    Kinematic_Constraints kinematic_constraints;
};

struct Navigation_State {
    Velocity2d velocity;
};

struct Motion_Step {
    Ecef &current;
    Ecef &next;
    Ecef &difference;
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
