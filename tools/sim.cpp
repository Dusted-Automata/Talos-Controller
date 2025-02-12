
#include "../src/trajectory.cpp"

#include "Eigen/src/Core/Matrix.h"
#include "raylib.h"
#include "raymath.h"
#include <stdio.h>

#define SCREEN_WIDTH 1000
#define SCREEN_HEIGHT 1000
#define ROBOT_SIZE 20.0

Vector3d baseArrow = Vector3d(1.0, 0.0, 0.0);
void SimRobot(Ecef_Coord &robotPos, Vector3d arrowVector,
              std::vector<Ecef_Coord> &waypoints,
              std::vector<Trajectory_Point> &trajectories, int &index) {
  if (trajectories[index].dt < GetTime())
    index++;
  // int index = std::floor(dt / 0.02);
  Vector3d movement;

  if (index < trajectories.size()) {
    Eigen::Matrix3d rotation =
        trajectories[index].pose.transformation_matrix.rotation();

    Vector3d translation =
        trajectories[index].pose.transformation_matrix.translation();
    movement = rotation * (trajectories[index].velocity.linear +
                           trajectories[index].velocity.angular);
    arrowVector = (baseArrow + translation);
    Linear_Velocity vel = movement * (double)GetFrameTime();
    robotPos.x() += vel.x();
    robotPos.y() -= vel.y();

    char robot[500];
    sprintf(robot, "x: %.2f y: %.2f index: %i  velocity.x: %f velocity.y: %f",
            robotPos.x(), robotPos.y(), index, vel.x(), vel.y());

    DrawText(robot, 10, 40, 20, BLACK);
  }
  DrawCircleV({(float)arrowVector.x(), (float)arrowVector.y()}, 0.3, ORANGE);

  double yaw = atan2(trajectories[index].pose.transformation_matrix(1, 0),
                     trajectories[index].pose.transformation_matrix(0, 0));
  Rectangle robot = {static_cast<float>(robotPos.x()),
                     static_cast<float>(robotPos.y()), 2.0, 1.0};
  Vector2 origin = {2.0 / 2.0, 1.0 / 2.0};
  DrawRectanglePro(robot, origin, yaw * (180 / M_PI), RED);
  DrawLineV({(float)robotPos.x(), (float)robotPos.y()},
            {static_cast<float>(robotPos.x() + movement.x()),
             static_cast<float>(robotPos.y() - movement.y())},
            GREEN);
  DrawCircleV({static_cast<float>(robotPos.x() + movement.x()),
               static_cast<float>(robotPos.y() - movement.y())},
              0.1, GREEN);
}

void DrawAbsoluteGrid(Camera2D camera, float gridStep) {
  Vector2 zero = {0, 0};
  Vector2 topLeft = GetScreenToWorld2D(zero, camera);
  Vector2 screen = {SCREEN_WIDTH, SCREEN_HEIGHT};
  Vector2 bottomRight = GetScreenToWorld2D(screen, camera);

  for (float x = floorf(topLeft.x / gridStep) * gridStep; x < bottomRight.x;
       x += gridStep) {
    Vector2 start = {x, topLeft.y};
    Vector2 end = {x, bottomRight.y};
    DrawLineV(start, end, (x == 0) ? RED : LIGHTGRAY);
  }

  for (float y = floorf(topLeft.y / gridStep) * gridStep; y < bottomRight.y;
       y += gridStep) {
    Vector2 start = {topLeft.x, y};
    Vector2 end = {bottomRight.x, y};
    DrawLineV(start, end, (y == 0) ? RED : LIGHTGRAY);
  }
}

int main() {
  InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Absolute Coordinate System");
  SetTargetFPS(60);

  float dt = 0;

  Robot_Config config = {.hz = 50,
                         .motion_constraints = {.max_velocity = 2.0,
                                                .standing_turn_velocity = 2.0,
                                                .max_acceleration = 0.5,
                                                .max_deceleration = 0.5,
                                                .max_jerk = 0.0,
                                                .corner_velocity = 0.0}

  };
  Velocity_Profile vel_profile = {.acceleration_rate = 35.0,
                                  .deceleration_rate = 10.0};

  // std::vector<Ecef_Coord> waypoints = {
  //     {4100175.625135626, 476368.7899695045, 4846344.356704135},
  //     {4100209.6729529747, 476361.2681338759, 4846316.478097512},
  //     {4100218.5394949187, 476445.5598077707, 4846300.796185957},
  //     {4100241.72195791, 476441.0557096391, 4846281.753675706}};

  std::vector<Ecef_Coord> waypoints = {{0.0, 0.0, 0.0},
                                       {10.0, 0.0, 0.0},
                                       {10.0, 10.0, 0.0},
                                       {0.0, 10.0, 0.0},
                                       {0.0, 0.0, 0.0}};

  Trajectory_Controller controller(config.motion_constraints, vel_profile,
                                   config.hz);

  std::vector<Trajectory_Point> trajectories =
      controller.generate_trajectory(waypoints, config);

  // std::vector<double> velocities =
  //     generate_velocity_profile(trajectories, robot_config);
  //
  Ecef_Coord robotPos = waypoints[0];
  Vector3d arrowVector = waypoints[0];

  Camera2D camera = {
      .target = {(float)waypoints[0].x(), (float)waypoints[0].y()}};
  // camera.target = (Vector2){waypoints[0].x, waypoints[0].y};
  camera.offset = (Vector2){SCREEN_WIDTH / 2.0f, SCREEN_HEIGHT / 2.0f};
  camera.zoom = 1.0f;
  int index = 0;

  while (!WindowShouldClose()) {
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
      Vector2 delta = GetMouseDelta();
      delta = Vector2Scale(delta, -1.0f / camera.zoom);
      camera.target = Vector2Add(camera.target, delta);
    }

    float wheel = GetMouseWheelMove();
    if (wheel != 0) {
      Vector2 mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), camera);
      camera.offset = GetMousePosition();
      camera.target = mouseWorldPos;

      float scaleFactor = 1.0f + (0.25f * fabsf(wheel));
      camera.zoom *= (wheel > 0) ? scaleFactor : 1.0f / scaleFactor;
      camera.zoom = Clamp(camera.zoom, 0.125f, 64.0f);
    }

    BeginDrawing();
    ClearBackground(RAYWHITE);

    BeginMode2D(camera);

    float gridStep = 5.0f;
    DrawAbsoluteGrid(camera, gridStep);

    for (int i = 0; i < waypoints.size(); i++) {
      DrawCircle(waypoints[i].x(), waypoints[i].y(), 2, BLUE);
    }

    for (int i = 0; i < trajectories.size() - 1; i++) {
      DrawLineV({(float)trajectories[i].pose.point.x(),
                 (float)trajectories[i].pose.point.y()},
                {(float)trajectories[i + 1].pose.point.x(),
                 (float)trajectories[i + 1].pose.point.y()},
                GREEN);
    }

    SimRobot(robotPos, arrowVector, waypoints, trajectories, index);

    EndMode2D();

    Vector2 mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), camera);
    char coordText[500];
    sprintf(coordText, "World: (%.1f, %.1f) Zoom: %.2f Time: %f",
            mouseWorldPos.x, -mouseWorldPos.y, camera.zoom, dt);
    DrawText(coordText, 10, 10, 20, BLACK);
    char testText[50];
    sprintf(testText, "sin: %.2f cos: %.2f", sin(0.7854), cos(0.7854));
    DrawText(testText, 10, 40, 20, BLACK);
    // char robotTraj[50];
    // sprintf(testText, "traj: %f len: %zu", trajectories[1000].pose.theta,
    //         trajectories.size());
    // DrawText(testText, 10, 70, 20, BLACK);

    EndDrawing();
    dt = GetTime();
  }

  CloseWindow();
  saveToFile("waypoints", waypoints);
  saveToFile("trajectories", trajectories);
  // saveToFile("velocities", velocities);
  // saveToFile("time", trajectories.size());
  return 0;
}
