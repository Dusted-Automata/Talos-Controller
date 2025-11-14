#include "sensor.hpp"
#include "json.hpp"
#include "math.hpp"
#include <arpa/inet.h>
#include <iostream>
#include <thread>


using nlohmann::json;
void 
from_json(const json &j, navigation_msg &msg) {
    std::array<f64, 3> attitude = j.at("attitude").get<std::array<f64, 3>>();
    std::array<f64, 3> position = j.at("position").get<std::array<f64, 3>>();
    std::array<f64, 3> velocity = j.at("velocity").get<std::array<f64, 3>>();

    msg.time = j.at("time").get<f64>();
    msg.heading_roll = attitude[0];
    msg.heading_pitch = attitude[1];
    // double euler_angle_heading_yaw = attitude[2];
    // double radian_heading_yaw = to_radian(euler_angle_heading_yaw);
    double radian_heading_yaw = attitude[2];
    msg.heading_yaw = convert_to_positive_radians(M_PI / 2 - radian_heading_yaw); // Converting from
                                                                                  // NED to ENU
    msg.llh.lat() = position[0];
    msg.llh.lon() = position[1];
    msg.llh.alt() = position[2];

    msg.linear_velocity.x() = velocity[0];
    msg.linear_velocity.y() = velocity[1];
    msg.linear_velocity.z() = velocity[2];

}

void
sensor_client_init(Navigation_Sensor& sensor, const char* target_IP, u16 target_port ){
    sensor.fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sensor.fd  < 0) {
        perror("socket creation failed!");
        return;
    }

    // sensor.local_addr.sin_family = AF_INET;
    // sensor.local_addr.sin_addr.s_addr = INADDR_ANY;  
    // sensor.local_addr.sin_port = htons(local_port);
    //
    // if (bind(sensor.fd, (sockaddr*)&sensor.local_addr, sizeof(sensor.local_addr)) < 0) {
    //     perror("bind");
    //     close(sensor.fd);
    // }

    sensor.server_addr.sin_family = AF_INET;
    sensor.server_addr.sin_port = htons(target_port);
    sensor.server_addr.sin_addr.s_addr = inet_addr(target_IP);

    std::string msg = "Connect";
    socklen_t addr_len = sizeof(sensor.server_addr);

    sendto(sensor.fd, msg.c_str(), msg.length(), 0, (sockaddr*)&sensor.server_addr, addr_len);
}

void
sensor_loop(Navigation_Sensor &sensor)
{

    while (sensor.running) {
        char msg[1024];
        sockaddr_in senderAddr{};
        socklen_t senderLen = sizeof(senderAddr);

        const int bytesReceived = recvfrom(sensor.fd, msg, sizeof(msg), 0,
            reinterpret_cast<sockaddr *>(&senderAddr), &senderLen);

        if (bytesReceived < 0) {
            // Handle recv error
            if (errno == EINTR) {
                // Interrupted system call, just try again
                continue;
            }

            perror("recvfrom");
            continue; // don't kill the loop because of one bad recv
        }

        if (bytesReceived == 0) {
            // empty datagram – usually just ignore
            continue;
        }


        try {
            // Parse JSON once
            nlohmann::json j = nlohmann::json::parse(msg);
            std::cout << j.dump() << std::endl;

            // Convert to your typed message
            sensor.msg = j.get<navigation_msg>();

            if (sensor.msg->llh.lat() == 0.0 || sensor.msg->llh.lon() == 0.0 || sensor.msg->llh.alt() == 0.0) {
                sensor.msg.reset(); 
            } 

        } catch (const nlohmann::json::parse_error &e) {
            std::cerr << "JSON parse error at byte " << e.byte << ": " << e.what() << '\n';
            continue;

        } catch (const nlohmann::json::exception &e) {
            std::cerr << "Other JSON error: " << e.what() << '\n';
            continue;
        }
    }
}

// void sensor_client_deinit(){};
//
bool
sensor_start(Navigation_Sensor &sensor)
{
    if (sensor.running) return true;
    if (sensor.fd < 0) {
        return false;
    }

    sensor.recv_thread = std::thread(&sensor_loop, std::ref(sensor));
    sensor.running = true;
    return true;
}


void
update_pvat(Navigation_Sensor &sensor, Frames &frames)
{
    std::unique_lock<std::mutex> lock(sensor.mutex);
    if (sensor.msg.has_value()) {
        frames_update_based_on_measurement(frames, sensor.msg->llh);
        if (sensor.msg->heading_yaw >= 0.0) { 
            Eigen::Affine3d rotationMatrix;
            rotationMatrix = Eigen::AngleAxisd(sensor.msg->heading_yaw, Eigen::Vector3d::UnitZ());
            frames.local_frame.orientation = rotationMatrix; 
        }
    }
    sensor.msg.reset();
}

