
#include "ublox.hpp"
#include "sensor.hpp"
#include <arpa/inet.h>
#include <iostream>
#include <netinet/in.h>
#include <unistd.h>
void
Ublox::loop()
{

    while (!(socket.get_fd() < 0) && running) {
        if (!socket.recv(buf)) {
            running = false;
            socket.disconnect();
        }
        for (size_t i = 0; i < buf.count(); i++) {
            if (buf[i] == '\n') {
                int len = i + 1; // i + 1 to include the newline
                std::string msg(len, '\0');
                buf.read(std::span(msg.data(), len));
                i = 0;
                json j;
                try {
                    j = json::parse(msg);
                } catch (nlohmann::json::parse_error &e) {
                    std::cerr << "Parse error at byte " << e.byte << ": " << e.what() << std::endl;
                    std::cout << msg.size() << " | " << msg.substr((e.byte - 10), 20) << std::endl;
                    std::cout << msg << std::endl;
                    break;
                } catch (nlohmann::json::exception &e) {
                    std::cerr << "Other JSON error: " << e.what() << std::endl;
                    break;
                }

                if (j["identity"] == "GPGGA" || j["identity"] == "NAV-ATT") {
                    if (j["identity"] == "GPGGA") {
                        gga = GGA(j);
                    }
                    if (j["identity"] == "NAV-ATT") {
                        nav_att = Nav_Att(j);
                    }
                }
            }
        }
    }
}

bool
Ublox::start()
{

    std::cerr << "Starting Ublox" << std::endl;
    if (running) return true;
    if (!socket.connect()) {
        std::cerr << "Ublx couldn't connect to socket" << std::endl;
        return false;
    };

    if (socket.get_fd() < 0) {
        return false;
    }

    if (sensor_thread.joinable()) {
        sensor_thread.join();
    }

    sensor_thread = std::thread(&Sensor::loop, this);
    running = true;
    return true;
}

void
Ublox::consume(Msg_Type msg)
{
    switch (msg) {
    case Msg_Type::NAV_ATT: nav_att.reset(); return;
    case Msg_Type::GP_GGA: gga.reset(); return;
    }
}

template<>
std::optional<Nav_Att>
Ublox::get_latest(Msg_Type msg)
{
    // std::unique_lock<std::mutex> lock(sensor_mutex);
    if (msg == NAV_ATT) {
        return nav_att;
    }
    return std::nullopt;
}

template<>
std::optional<GGA>
Ublox::get_latest(Msg_Type msg)
{
    // std::unique_lock<std::mutex> lock(sensor_mutex);
    if (msg == GP_GGA) {
        return gga;
    }
    return std::nullopt;
}

inline bool
above_epsilon(double lat, double lng, double alt)
{
    if (std::abs(lat) > 0.001 || std::abs(lng) > 0.001 || std::abs(alt) > 0.001) {
        return true;
    }
    return false;
}

void
update_position(Ublox &ublox, Frames &frames)
{
    auto ublox_gga = ublox.get_latest<GGA>(Msg_Type::GP_GGA);
    if (ublox_gga.has_value()) {
        GGA val = ublox_gga.value();
        double lat = to_radian(val.latlng.lat);
        double lng = to_radian(val.latlng.lng);
        double alt = val.alt;

        if (above_epsilon(lat, lng, alt)) {
            // Vector3d error_vec = robot->frames.get_error_vector_in_NED(lat, lng, alt);
            frames.update_based_on_measurement({ lat, lng, alt });
        }
        ublox.consume(Msg_Type::GP_GGA);
    }
}
void
update_heading(Ublox &ublox, Frames &frames)
{

    auto ublox_simple = ublox.get_latest<Nav_Att>(Msg_Type::NAV_ATT);
    if (ublox_simple.has_value()) {
        Nav_Att nav_att = ublox_simple.value();
        double heading = to_radian(nav_att.heading);
        std::cout << nav_att.heading << std::endl;
        // std::cout << heading << std::endl;
        Eigen::AngleAxisd yawAngle(heading, Eigen::Vector3d::UnitZ());
        Eigen::Matrix3d rotationMatrix = yawAngle.toRotationMatrix();
        frames.local_frame.orientation.linear() + rotationMatrix;
        ublox.consume(Msg_Type::NAV_ATT);
    }
}
