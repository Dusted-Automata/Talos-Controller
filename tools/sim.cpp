
#include "../trajectory.cpp"

#include "raylib.h"
#include "raymath.h"
#include <stdio.h>

#define SCREEN_WIDTH 1000
#define SCREEN_HEIGHT 1000

void SimRobot(Ecef_Coord &robotPos, std::vector<Ecef_Coord> &waypoints,
              std::vector<Trajectory_Point> &trajectories,
              std::vector<double> &velocities, double dt) {

  float val = GetTime();
  // val = ((int)(val * 90) % 360);
  // float x = PI * ((int)val % 360);
  int index = std::floor(dt / 0.02);
  float vel = velocities[index] * 0.02;
  if (index < trajectories.size()) {
    float th = trajectories[index].pose.theta;
    robotPos.x += cos(th) * (vel);
    robotPos.y += sin(th) * (vel);

    char robot[500];
    sprintf(robot, "x: %.2f y: %.2f index: %i theta: %f velocity: %f",
            robotPos.x, robotPos.y, index, th, vel);

    DrawText(robot, 10, 40, 20, BLACK);
  }
  DrawCircleV({robotPos.x, robotPos.y}, 0.1, RED);
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

  Camera2D camera = {0};
  camera.target = (Vector2){0, 0};
  camera.offset = (Vector2){SCREEN_WIDTH / 2.0f, SCREEN_HEIGHT / 2.0f};
  camera.zoom = 1.0f;

  float dt = 0;

  std::vector<Ecef_Coord> waypoints = {
      {0.0, 0.0}, {10.0, -10.0}, {20.0, 0.0}, {30.0, -20.0}};

  std::vector<Trajectory_Point> trajectories = {};
  generate_trajectory(trajectories, waypoints, 0.02);

  std::vector<double> velocities =
      generate_velocity_profile(trajectories, 1.0, 0.5);

  Ecef_Coord robotPos = {0.0, 0.0};

  while (!WindowShouldClose()) {
    // Camera movement
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
      Vector2 delta = GetMouseDelta();
      delta = Vector2Scale(delta, -1.0f / camera.zoom);
      camera.target = Vector2Add(camera.target, delta);
    }

    // Zoom
    float wheel = GetMouseWheelMove();
    if (wheel != 0) {
      Vector2 mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), camera);
      camera.offset = GetMousePosition();
      camera.target = mouseWorldPos;

      float scaleFactor = 1.0f + (0.25f * fabsf(wheel));
      camera.zoom *= (wheel > 0) ? scaleFactor : 1.0f / scaleFactor;
      camera.zoom = Clamp(camera.zoom, 0.125f, 64.0f);
    }

    // Drawing
    BeginDrawing();
    ClearBackground(RAYWHITE);

    BeginMode2D(camera);

    // Draw absolute grid
    float gridStep = 5.0f;
    DrawAbsoluteGrid(camera, gridStep);

    // Draw origin point

    for (int i = 0; i < waypoints.size(); i++) {
      DrawCircle(waypoints[i].x, waypoints[i].y, 2, BLUE);
    }

    for (int i = 0; i < trajectories.size() - 1; i++) {
      DrawLineV({trajectories[i].pose.x, trajectories[i].pose.y},
                {trajectories[i + 1].pose.x, trajectories[i + 1].pose.y},
                GREEN);
    }

    SimRobot(robotPos, waypoints, trajectories, velocities, GetTime());

    EndMode2D();

    // Draw mouse coordinates in world space
    Vector2 mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), camera);
    char coordText[500];
    sprintf(coordText, "World: (%.1f, %.1f) Zoom: %.2f Time: %f",
            mouseWorldPos.x, -mouseWorldPos.y, camera.zoom, dt);
    DrawText(coordText, 10, 10, 20, BLACK);
    char testText[50];
    sprintf(testText, "sin: %.2f cos: %.2f", sin(0.7854), cos(0.7854));
    DrawText(testText, 10, 40, 20, BLACK);
    char robotTraj[50];
    sprintf(testText, "traj: %f len: %zu", trajectories[1000].pose.theta,
            trajectories.size());
    DrawText(testText, 10, 70, 20, BLACK);

    EndDrawing();
    dt = GetTime();
  }

  CloseWindow();
  saveToFile("waypoints", waypoints);
  saveToFile("trajectories", trajectories);
  saveToFile("velocities", velocities);
  return 0;
}
