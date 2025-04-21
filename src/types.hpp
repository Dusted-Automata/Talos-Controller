#pragma once
#include <Eigen/Dense>
#include <array>
#include <memory>

using Eigen::Affine3d;
using Eigen::Matrix4d;
using Eigen::Vector3d;
typedef Vector3d Ecef_Coord;
typedef Vector3d LLH;
typedef Vector3d Linear_Velocity;
typedef Vector3d Angular_Velocity;

struct Robot_State
{
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
struct Velocity2d
{
    // high-level representation
    // can have 12 joint angles/torques (3 per leg)
    Linear_Velocity linear;   // x, y, z velocity (m/s)
    Angular_Velocity angular; // roll, pitch, yaw rates (rad/s)
};

struct Pose_State
{
    Ecef_Coord position; // x, y, z
    Affine3d orientation;
    // Eigen::Quaterniond orientation; // quaternion
    Velocity2d velocity; // vx, vy, vz \  wx, wy, wz
    double dt;
};

struct Pose
{
    Ecef_Coord point;
    Affine3d transformation_matrix; // Change this to Quaternion maybe
};

struct Motion_Constraints
{
    double max_velocity;
    double min_velocity;
    double max_acceleration;
    double max_deceleration;
    double max_jerk;
};

struct Robot_Config
{
    int hz;
    Motion_Constraints motion_constraints;
};

struct Motion_Step
{
    Ecef_Coord &current;
    Ecef_Coord &next;
    Ecef_Coord &difference;
    Affine3d &robot_frame;
    double &dt;
};

struct Trajectory_Point
{
    Pose pose;
    double dt;
    Velocity2d velocity;
};

template <typename T>
class Ring_Buffer {
private:
    std::unique_ptr<T[]> buffer;
    size_t capacity;
    size_t head = 0;
    size_t tail = 0;
    size_t num_items = 0;
    bool full = false;

public:
  explicit Ring_Buffer(size_t size) : capacity(size), buffer(std::make_unique<T[]>(size))
  {
      if (size == 0)
          throw std::invalid_argument("Buffer size cannot be zero");
  }

  T &operator[](size_t index)
  {
      if (index >= num_items)
      {
          throw std::out_of_range("Index out of range");
      }

      size_t actual_index = (tail + index) % capacity;
      return buffer[actual_index];
  }

  const T &operator[](size_t index) const
  {
      if (index >= num_items)
      {
          throw std::out_of_range("Index out of range");
      }
      size_t actual_index = (head + index) % capacity;
      return buffer[actual_index];
  }

  bool isEmpty() const { return (!full && (head == tail)); }

  bool isFull() const { return full; }

  size_t count() const { return num_items; }

  size_t getCapacity() const { return capacity; }

  bool push(const T &item)
  {
      if (full)
          return false;

      buffer[head] = item;
      tail = (head + 1) % capacity;
      num_items++;
      full = (head == tail);
      return true;
  }

  bool pop(T &item)
  {
      if (isEmpty())
          return false;

      item = buffer[tail];
      head = (tail + 1) % capacity;
      num_items--;
      full = false;
      return true;
  }

  bool write(const T *foreign_buf, size_t amount)
  {
      if (capacity - num_items < amount)
          return false;

      size_t contiguous_space = capacity - head;
      if (amount <= contiguous_space)
      {
          std::memcpy(&buffer[head], foreign_buf, amount * sizeof(T));
      }
      else
      {
          std::memcpy(&buffer[head], foreign_buf, contiguous_space * sizeof(T));
          std::memcpy(&buffer[0], foreign_buf + contiguous_space,
                      (amount - contiguous_space) * sizeof(T));
      }

      head = (head + amount) % capacity;
      num_items += amount;
      full = (head == tail);
      return true;
  }

  // Read multiple elements from the buffer using memcpy
  bool read(T *foreign_buf, size_t amount)
  {
      if (num_items < amount)
          return false;

      // Calculate contiguous elements available at head
      size_t contiguous_elements = capacity - tail;

      if (amount <= contiguous_elements)
      {
          std::memcpy(foreign_buf, &buffer[tail], amount * sizeof(T));
      }
      else
      {
          std::memcpy(foreign_buf, &buffer[tail], contiguous_elements * sizeof(T));
          std::memcpy(foreign_buf + contiguous_elements, &buffer[0],
                      (amount - contiguous_elements) * sizeof(T));
      }

      tail = (tail + amount) % capacity;
      num_items -= amount;
      full = false;
      return true;
  }

  bool peek(T &item) const
  {
      if (isEmpty())
          return false;
      item = buffer[head];
      return true;
  }

  void clear()
  {
      head = tail = 0;
      full = false;
      num_items = 0;
  }
};

