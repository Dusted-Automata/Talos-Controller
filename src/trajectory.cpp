#include "trajectory.hpp"
#include <cmath>
#include <fstream>
#include <iostream>

double compute_displacement(double init_velocity, double final_velocity, double acceleration) {
  double v0 = (init_velocity * init_velocity);
  double vf = (final_velocity * final_velocity);
  double avg_accel = (2 * acceleration);
  double displacement = (vf - v0) / avg_accel;
  return std::abs(displacement);
}

Trajectory_Point Trajectory_Controller::trajectory_turn(Motion_Step step) {
  Ecef_Coord difference = step.next - step.current;
  double azimuth_rad_world = atan2(step.difference.y(), step.difference.x());
  double azimuth_rad_robot = atan2(step.robot_frame(1, 0), step.robot_frame(0, 0));
  double azimuth_rad = azimuth_rad_world - azimuth_rad_robot;
  if (azimuth_rad > M_PI)
    azimuth_rad -= 2 * M_PI;
  if (azimuth_rad < -M_PI)
    azimuth_rad += 2 * M_PI;
  // std::cout << "azimuth_rad_world: " << azimuth_rad_world
  //           << " azimuth_rad_robot: " << azimuth_rad_robot
  //           << " azimuth_rad: " << azimuth_rad << std::endl;

  Angular_Velocity angular;
  angular.setZero();
  angular.z() = config.motion_constraints.standing_turn_velocity;
  step.robot_frame.rotate(Eigen::AngleAxisd((azimuth_rad), Vector3d::UnitZ()));
  Velocity2d velocity = {.linear = {}, .angular = angular};
  Affine3d transformation = step.robot_frame;
  Pose pose = {.point = step.current, .transformation_matrix = transformation};
  double turn_duration = azimuth_rad / config.motion_constraints.standing_turn_velocity;
  step.dt += std::abs(turn_duration);
  Trajectory_Point tp = {.pose = pose, .dt = step.dt, .velocity = velocity};
  return tp;
}

std::vector<Trajectory_Point> Trajectory_Controller::trajectory_ramp_up(Motion_Step step) {
  // TODO:
  // Not quite correct. I don't know if I will even reach max_velocity
  std::vector<Trajectory_Point> trajectory;
  double distance_acceleration = compute_displacement(0, config.motion_constraints.max_velocity,
                                                      config.velocity_profile.acceleration_rate);
  double time_accelerating =
      config.motion_constraints.max_velocity / config.velocity_profile.acceleration_rate;

  Vector3d unit_vector = step.difference.normalized();

  for (int j = 0; j <= sampling_rate; j++) {
    double t = static_cast<double>(j) / sampling_rate;
    Ecef_Coord next_point = step.current + ((t * distance_acceleration) * unit_vector);
    Vector3d linear((config.motion_constraints.max_velocity * t), 0.0, 0.0);
    step.dt += (time_accelerating * t);
    Trajectory_Point tp =
        new_trajectory_point(step.robot_frame, next_point, linear, {0.0, 0.0, 0.0}, step.dt);
    trajectory.push_back(tp);
  }
  return trajectory;
}

Trajectory_Point Trajectory_Controller::trajectory_cruise(Motion_Step step) {
  Vector3d unit_vector = step.difference.normalized();
  double distance_acceleration = compute_displacement(0, config.motion_constraints.max_velocity,
                                                      config.velocity_profile.acceleration_rate);
  double distance_deceleration = compute_displacement(config.motion_constraints.max_velocity, 0,
                                                      config.velocity_profile.acceleration_rate);
  double distance_cruising =
      step.difference.norm() - (distance_acceleration + distance_deceleration);

  // std::cout << distance_acceleration << std::endl;
  double time_cruising = distance_cruising / config.motion_constraints.max_velocity;

  Ecef_Coord next_point = step.next - (distance_deceleration * unit_vector);
  Vector3d linear((config.motion_constraints.max_velocity), 0.0, 0.0);
  step.dt += time_cruising;
  Trajectory_Point tp =
      new_trajectory_point(step.robot_frame, next_point, linear, {0.0, 0.0, 0.0}, step.dt);
  return tp;
}

std::vector<Trajectory_Point> Trajectory_Controller::trajectory_ramp_down(Motion_Step step) {
  std::vector<Trajectory_Point> trajectory;
  double distance_acceleration = compute_displacement(0, config.motion_constraints.max_velocity,
                                                      config.velocity_profile.acceleration_rate);

  double distance_deceleration = compute_displacement(config.motion_constraints.max_velocity, 0,
                                                      config.velocity_profile.acceleration_rate);
  double time_accelerating;
  time_accelerating =
      config.motion_constraints.max_velocity / config.velocity_profile.acceleration_rate;
  double time_decelerating;

  Vector3d unit_vector = step.difference.normalized();

  // time_decelerating t = -max_velocity / -acceleartion_rate
  // - acceleration is deceleration, but i am unsure if i should codify
  // that like that.
  time_decelerating =
      config.motion_constraints.max_velocity / config.velocity_profile.deceleration_rate;
  for (int j = 0; j <= sampling_rate; j++) {
    double t = static_cast<double>(j) / sampling_rate;
    Ecef_Coord next_point = step.current + ((t * distance_deceleration) * unit_vector);
    Vector3d linear((config.motion_constraints.max_velocity * (1 - t)), 0.0, 0.0);
    step.dt += (time_accelerating * t);
    Trajectory_Point tp =
        new_trajectory_point(step.robot_frame, next_point, linear, {0.0, 0.0, 0.0}, step.dt);
    trajectory.push_back(tp);
  }
  return trajectory;
}

Trajectory_Point Trajectory_Controller::new_trajectory_point(Affine3d &robot_frame,
                                                             Ecef_Coord next_point, Vector3d linear,
                                                             Vector3d angular, double dt) {
  Ecef_Coord diff_vec = next_point - robot_frame.translation();
  robot_frame.translation() += diff_vec;
  Velocity2d velocity = {.linear = linear, .angular = angular};
  Affine3d transformation = robot_frame;
  Pose pose = {.point = next_point, .transformation_matrix = transformation};
  Trajectory_Point tp = {.pose = pose, .dt = dt, .velocity = velocity};
  return tp;
}

std::vector<Trajectory_Point> Trajectory_Controller::generate_trajectory(Ecef_Coord current,
                                                                         Ecef_Coord next) {
  std::vector<Trajectory_Point> trajectory = {};
  Affine3d &robot_frame = config.transform_world_to_robot;

  double dt = 0.0;
  Ecef_Coord difference = next - current;
  Motion_Step step = {.current = current,
                      .next = next,
                      .difference = difference,
                      .robot_frame = robot_frame,
                      .dt = dt};

  { // Turning
    Trajectory_Point tp = trajectory_turn(step);
    trajectory.push_back(tp);
  }

  { // RAMP UP
    std::vector<Trajectory_Point> ramp_up = trajectory_ramp_up(step);
    trajectory.insert(trajectory.end(), ramp_up.begin(), ramp_up.end());
  }

  { // CRUISE
    Trajectory_Point tp = trajectory_cruise(step);
    trajectory.push_back(tp);
  }
  step.current = trajectory.back().pose.point;
  { // RAMP DOWN
    std::vector<Trajectory_Point> ramp_down = trajectory_ramp_down(step);
    trajectory.insert(trajectory.end(), ramp_down.begin(), ramp_down.end());
  }
  return trajectory;
}

Pose Trajectory_Controller::get_current_pose() { return {}; }

Velocity2d
Trajectory_Controller::follow_trajectory(Thread_Safe_Queue<Trajectory_Point> &trajectories,
                                         Robot_State &state) {
  Velocity2d cmd = {.linear = Linear_Velocity().setZero(), .angular = Angular_Velocity().setZero()};

  std::optional<Trajectory_Point> point = trajectories.front();

  if (!point) {
    linear_pid.reset();
    angular_pid.reset();
    trajectory_time = 0.0;
    // empty command
    return cmd;
  }

  // TODO: If the robot is still not done moving after the calculated dt, then
  // it should still move.
  if (point.value().dt < trajectory_time) {
    trajectories.pop();
    // unwrap
    point = trajectories.front();
    if (!point)
      return cmd;
    linear_pid.setpoint = point.value().velocity.linear.x();
    angular_pid.setpoint = point.value().velocity.angular.z();
  }

  std::cout << "DT: " << point.value().dt << " | t_dt: " << trajectory_time
            << " lin: " << point.value().velocity.linear.x()
            << " ang: " << point.value().velocity.angular.z() << std::endl;

  cmd.linear.x() = linear_pid.update(state.velocity[0], trajectory_time);
  cmd.angular.z() = angular_pid.update(state.yawSpeed, trajectory_time);
  // TODO: HZ from Robot
  trajectory_time += 0.002;
  return cmd;
}

void Trajectory_Controller::trajectory_loop(Thread_Safe_Queue<Trajectory_Point> &trajectories,
                                            Thread_Safe_Queue<Ecef_Coord> &waypoints) {
  std::optional<std::pair<Ecef_Coord, Ecef_Coord>> path = waypoints.front_two();
  if (trajectories.empty()) {
    if (path.has_value()) {
      std::cout << "EMPTY TRAJECTORIES" << std::endl;
      std::vector<Trajectory_Point> trajectory =
          generate_trajectory(path.value().first, path.value().second);
      for (auto &point : trajectory) {
        trajectories.push(point);
      }
      waypoints.pop();
    }
  }
}

void Trajectory_Controller::path_loop(Thread_Safe_Queue<Ecef_Coord> &path,
                                      std::vector<Ecef_Coord> &waypoints) {
  if (waypoints.empty()) {
    return;
  }
  if (!added_paths) {
    std::cout << "EMPTY FIRST WAYPOINTS" << std::endl;
    for (Ecef_Coord &waypoint : waypoints) {
      std::cout << "EMPTY TRAJECTORIES" << std::endl;
      path.push(waypoint);
    }
    added_paths = true;
  }

  // FIXME: One waypoint does not get popped off, so it wont loop
  if (path.empty() && path_looping) {
    std::cout << "EMPTY WAYPOINTS" << std::endl;
    for (Ecef_Coord &waypoint : waypoints) {
      path.push(waypoint);
    }
  }
}

void Trajectory_Controller::local_replanning() {}

static bool saveToFile(const std::string &filename, const std::vector<Ecef_Coord> &data) {
  std::ofstream outFile(filename);
  if (!outFile.is_open()) {
    std::cerr << "Unable to open file for writing: " << filename << std::endl;
    return false;
  }

  for (const auto &line : data) {
    outFile << line.x() << "," << line.y() << "," << line.z() << std::endl;
  }

  outFile.close();
  return true;
}

static bool saveToFile(const std::string &filename, const std::vector<Trajectory_Point> &data) {
  std::ofstream outFile(filename);
  if (!outFile.is_open()) {
    std::cerr << "Unable to open file for writing: " << filename << std::endl;
    return false;
  }

  for (const auto &line : data) {
    outFile << std::fixed;
    outFile << " ------------------------------------------------- " << std::endl;
    outFile << "dt: " << line.dt << " " << std::endl;
    outFile << "pose: " << line.pose.point.x() << " " << line.pose.point.y() << " "
            << line.pose.point.z() << " " << std::endl;
    outFile << "Velocity-linear: forward: " << line.velocity.linear.x()
            << " lateral: " << line.velocity.linear.y() << " vertical: " << line.velocity.linear.z()
            << std::endl;
    outFile << "Velocity-angular: pitch: " << line.velocity.angular.x()
            << " roll: " << line.velocity.angular.y() << " yaw: " << line.velocity.angular.z()
            << std::endl;
    // outFile << " ------------------------------------------------- "
    // << std::endl;
  }

  outFile.close();
  return true;
}
