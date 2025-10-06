#pragma once

#include "pid.hpp"
#include <filesystem>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wextra"
#pragma GCC diagnostic ignored "-Wconversion"
// #pragma GCC diagnostic ignored "-Wclass-memaccess"
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
typedef Vector3d Velocity;
typedef Vector3d Acceleration;

#include <float.h>
#include <limits.h>
#include <stddef.h>
#include <stdint.h>

typedef int8_t int8;
typedef int16_t int16;
typedef int32_t int32;
typedef int64_t int64;
typedef int32 bool32;

typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef uint64_t uint64;

typedef intptr_t intptr;
typedef uintptr_t uintptr;

typedef size_t memory_index;

typedef float real32;
typedef double real64;

typedef int8 i8;
typedef int8 s08;
typedef int8 b8;

typedef int16 i16;
typedef int32 i32;
typedef int64 i64;

typedef uint8 u8;
typedef uint8 u08;
typedef uint16 u16;
typedef uint32 u32;
typedef uint64 u64;

typedef real32 f32;
typedef real32 r32;
typedef real64 f64;
typedef real64 r64;


typedef uintptr_t umm;
typedef intptr_t smm;

#define U8Max 255
#define U16Max 65535
#define S32Min ((s32)0x80000000)
#define S32Max ((s32)0x7fffffff)
#define U32Min 0
#define U32Max ((u32) - 1)
#define U64Max ((u64) - 1)
#define F32Max FLT_MAX
#define F32Min -FLT_MAX

#define OffsetOf(type, Member) (umm) & (((type *)0)->Member)

#define FILE_AND_LINE__(A, B) A "|" #B
#define FILE_AND_LINE_(A, B) FILE_AND_LINE__(A, B)
#define FILE_AND_LINE FILE_AND_LINE_(__FILE__, __LINE__)

struct buffer {
    umm Count;
    u8 *Data;
};

typedef struct buffer string;

// typedef union v2 {
//     struct {
//         f32 x, y;
//     };
//     struct {
//         f32 u, v;
//     };
//     struct {
//         f32 Width, Height;
//     };
//     f32 E[2];
// } v2;
//
// union v2u {
//     struct {
//         u32 x, y;
//     };
//     struct {
//         u32 Width, Height;
//     };
//     u32 E[2];
// };
//
// union v2s {
//     struct {
//         s32 x, y;
//     };
//     s32 E[2];
// };
//
// typedef union v3 {
//     struct {
//         f32 x, y, z;
//     };
//     struct {
//         f32 u, v, __;
//     };
//     struct {
//         f32 r, g, b;
//     };
//     struct {
//         v2 xy;
//         f32 Ignored0_;
//     };
//     struct {
//         f32 Ignored1_;
//         v2 yz;
//     };
//     struct {
//         v2 uv;
//         f32 Ignored2_;
//     };
//     struct {
//         f32 Ignored3_;
//         v2 v__;
//     };
//     f32 E[3];
// } v3;
//
// union v3s {
//     struct {
//         s32 x;
//         s32 y;
//         s32 z;
//     };
//     s32 E[3];
// };
//
// union v4 {
//     struct {
//         union {
//             v3 xyz;
//             struct {
//                 f32 x, y, z;
//             };
//         };
//
//         f32 w;
//     };
//     struct {
//         union {
//             v3 rgb;
//             struct {
//                 f32 r, g, b;
//             };
//         };
//
//         f32 a;
//     };
//     struct {
//         v2 xy;
//         f32 Ignored0_;
//         f32 Ignored1_;
//     };
//     struct {
//         f32 Ignored2_;
//         v2 yz;
//         f32 Ignored3_;
//     };
//     struct {
//         f32 Ignored4_;
//         f32 Ignored5_;
//         v2 zw;
//     };
//     f32 E[4];
// };

// #define function static

// #if !defined(internal)
// #define internal static
// #endif
#define local_persist static
#define global static
#define TEMPORARY 


#define Pi32 3.14159265359f
#define Tau32 6.28318530717958647692f

#define Kilobytes(Value) ((Value) * 1024LL)
#define Megabytes(Value) (Kilobytes(Value) * 1024LL)
#define Gigabytes(Value) (Megabytes(Value) * 1024LL)
#define Terabytes(Value) (Gigabytes(Value) * 1024LL)

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
    east()
    {
        return v.x();
    }

    double &
    north()
    {
        return v.y();
    }

    double &
    up()
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
    Linear_Velocity linear_vel;   // x, y, z velocity (m/s)
    Angular_Velocity angular_vel; // roll, pitch, yaw rates (rad/s)
};

struct Linear {
    // high-level representation
    Velocity velocity; // (m/s)
    Acceleration acceleration; // (m/s^2)
};

struct Angular {
    // high-level representation
    Velocity velocity; // (rad/s)
    Acceleration acceleration; // (rad/s^2)
};

static inline double
to_radian(double degrees)
{
    double rad = degrees * (M_PI / 180.);
    // const double TWO_PI = 2.0 * M_PI;
    // rad = fmod(rad + M_PI, TWO_PI);
    // if (rad < 0) rad += TWO_PI;
    return rad;
}

static inline double
to_degrees(double rad)
{
    constexpr double RAD_TO_DEG = 180.0 / M_PI;
    return rad * RAD_TO_DEG;
};

struct Pose_State {
    Ecef position = Eigen::Vector3d(0, 0, 0); // x, y, z
    Affine3d orientation = Eigen::Affine3d::Identity();
    // Eigen::Matrix3d orientation = Eigen::Matrix3d::Identity();
    // Eigen::Quaterniond orientation; // quaternion
    Velocity2d velocity = { .linear_vel = Vector3d::Zero(),
        .angular_vel = Vector3d::Zero() }; // vx, vy, vz \  wx, wy, wz
    Linear linear = { .velocity = Vector3d::Zero(), .acceleration = Vector3d::Zero()};
    Angular angular = { .velocity = Vector3d::Zero(), .acceleration = Vector3d::Zero()};
    double dt;
};

struct Pose {
    Ecef point;
    ENU local_point;
    Affine3d transformation_matrix; // Change this to Quaternion maybe
};

struct LA {
    Linear linear;
    Angular angular;
};

struct PVA {
    Pose pose;
    // Velocity2d velocity;
    Linear linear;
    Angular angular;
};

// struct Kinematic_Constraints {
//     double v_max = 0;       // max linear velocity (m/s)
//     double v_min = 0;       // min linear velocity (m/s)
//     double omega_max = 0;   // max angular velocity (rad/s)
//     double omega_min = 0;   // min angular velocity (rad/s)
//     double a_max = 0;       // max linear acceleration (m/s^2)
//     double a_min = 0;       // max linear deceleration (m/s^2), possibly negative
//     double alpha_max = 0;   // max angular acceleration
//     double j_max = 0;       // max Linear jerk (m/s^3)
//     double j_omega_max = 0; // max Angular jerk (m/s^3)
// };

enum class Path_Direction {
    NORMAL,
    REVERSE,
    LOOP,
};

struct Path_Config {
    Path_Direction direction;
    std::filesystem::path filepath;
    double interpolation_distances_in_meters;
};

struct Kinematic_Constraints {
    double velocity_forward_max = 0;       // max linear velocity (m/s)
    double velocity_backward_max = 0;      // min linear velocity (m/s)
    double velocity_turning_left_max = 0;  // max angular velocity (rad/s)
    double velocity_turning_right_max = 0; // min angular velocity (rad/s)
    double acceleration_max = 0;           // max linear acceleration (m/s^2)
    double deceleration_max = 0;           // max linear deceleration (m/s^2), possibly negative
    double alpha_max = 0;                  // max angular acceleration
    double jerk_max = 0;                   // max Linear jerk (m/s^3)
    double jerk_omega_max = 0;             // max Angular jerk (m/s^3)
};

struct Robot_Config {
    Kinematic_Constraints kinematic_constraints;
    PIDGains linear_gains;
    PIDGains angular_gains;
    Path_Config path_config;
    double goal_tolerance_in_meters;
    int control_loop_hz;
};

struct Navigation_State {
    Velocity2d velocity;
};


template<typename T, std::size_t Capacity> class Ring_Buffer
{
  private:
    size_t head = 0;
    size_t tail = 0;
    size_t num_items = 0;
    bool full = false;
    std::array<T, Capacity> buffer = {};

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

    size_t
    contigues_space_from_head()
    {
        return std::min((buffer.size() - num_items), buffer.size() - head);
    }

    char *
    data()
    {
        return buffer.data();
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
