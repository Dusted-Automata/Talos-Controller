#pragma once

#include "frames.hpp"
#include "types.hpp"
#include <netinet/in.h>
#include <thread>

// struct jannick_msg {
//     std::array<f64, 3> position;
//     std::array<f64, 3> velocity;
//     std::array<f64, 3> attitude;
// };

struct navigation_msg {
    f64 heading_roll;
    f64 heading_pitch;
    f64 heading_yaw;
    LLH llh;
    Linear_Velocity linear_velocity; // TODO: currently is 0
    // Angular_Velocity angular_velocity; // TODO: Not yet implemented by Jannick
    f64 time;
};

struct Navigation_Sensor {
    std::atomic_bool running = false;
    std::thread recv_thread;
    navigation_msg old_msg;
    std::optional<navigation_msg> msg;
    sockaddr_in server_addr;
    // sockaddr_in local_addr;
    int fd;
    std::mutex mutex;

    // TCP_Client socket = TCP_Client("127.0.0.1", 50020);
    // Ring_Buffer<char, TCP_BUFFER_LENGTH * 2> buf;

};

void sensor_client_init(Navigation_Sensor& sensor, const char* target_IP, u16 target_port );
bool sensor_connect(Navigation_Sensor &sensor);
bool sensor_start(Navigation_Sensor &sensor);
void update_pvat(Navigation_Sensor &sensor, Frames &frames);

// void update_position(Ublox &ublox, Frames &frames);
// void update_heading(Ublox &ublox, Frames &frames);
