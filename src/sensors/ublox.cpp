
#include "ublox.hpp"
#include "cppmap3d.hh"
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

                std::string id = j["identity"];
                if (id == "GPGGA") {
                    // std::cout << j.dump(4) << std::endl;
                    gga = GGA(j);
                    // LatLng latlng = gga.value().latlng;
                    // LLH llh = LLH(latlng.lat, latlng.lng, 241);
                    // Ecef ecef = cppmap3d::geodetic2ecef(llh);
                }
                if (id == "NAV-ATT") {
                    // std::cout << j.dump(4) << std::endl;
                    nav_att = Nav_Att(j);
                }

                if (id == "NAV-PVAT") {
                    // std::cout << j.dump(4) << std::endl;
                    nav_pvat = Nav_Pvat(j);
                    // LLH llh = nav_pvat->llh;
                    // Ecef ecef = cppmap3d::geodetic2ecef(llh);
                    // std::cout << "ECEF: " << ecef.raw().transpose() << " | " << llh.raw().transpose() << std::endl;
                    if (nav_pvat.has_value()) {
                        // std::cout << nav_pvat->veh_heading << " | " << nav_pvat->mot_heading << " | "
                        //           << nav_pvat->accHeading << std::endl;
                    }
                }
            }
        }

        // std::optional<json> j = socket.recv();
        // if (j.has_value()) {
        //     if (j.value()["identity"] == "GPGGA") {
        //         std::cout << j.value().dump(4) << std::endl;
        //         gga = GGA(j.value());
        //     }
        //     if (j.value()["identity"] == "NAV-ATT") {
        //         std::cout << j.value().dump(4) << std::endl;
        //         nav_att = Nav_Att(j.value());
        //     }
        // }
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
    std::unique_lock<std::mutex> lock(sensor_mutex);
    switch (msg) {
    case Msg_Type::NAV_PVAT: nav_att.reset(); return;
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

template<>
std::optional<Nav_Pvat>
Ublox::get_latest(Msg_Type msg)
{
    // std::unique_lock<std::mutex> lock(sensor_mutex);
    if (msg == NAV_PVAT) {
        return nav_pvat;
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

        if (above_epsilon(val.llh.lat(), val.llh.lon(), val.llh.alt())) {
            // Vector3d error_vec = robot->frames.get_error_vector_in_NED(lat, lng, alt);
            frames_update_based_on_measurement(frames, val.llh);
        }
        ublox.consume(Msg_Type::GP_GGA);
    }
}

double
convert_to_positive_radians(double angle)
{

    if (angle < 0) {
        return angle + 2 * M_PI;
    }
    return angle;
}

double
min_angle_difference(double angle1, double angle2)
{
    // https://stackoverflow.com/questions/1878907/how-can-i-find-the-smallest-difference-between-two-angles-around-a-point
    // Angles must be subtracted differently than other values.
    return atan2(sin(angle2 - angle1), cos(angle2 - angle1));
}

void
update_heading(Ublox &ublox, Frames &frames, Heading &h)
{

    auto ublox_simple = ublox.get_latest<Nav_Pvat>(Msg_Type::NAV_PVAT);
    if (ublox_simple.has_value()) {
        Nav_Pvat nav_pvat = ublox_simple.value();
        std::cout << "heading_in_radians: " << h.initial_heading_in_radians << " | accuracy: " << nav_pvat.accHeading
                  << " | veh_heading: " << nav_pvat.veh_heading << std::endl;
        double heading = convert_to_positive_radians(nav_pvat.veh_heading);
        h.heading_accuracy = nav_pvat.accHeading;
        if (nav_pvat.accHeading < 30.0) { // NOTE: TBD
            Eigen::Affine3d rotationMatrix;

            rotationMatrix = Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ());
            frames.local_frame.orientation = rotationMatrix; // NOTE: To be checked!
        }
        ublox.consume(Msg_Type::NAV_PVAT);
    }
}

bool
Ublox::update_speed(Velocity2d vel)
{
    json j;
    j["identity"] = "ESF-MEAS-SPEED";
    j["speed"] = vel.linear_vel.norm();
    std::string msg = j.dump();
    msg.append("\n");
    // std::cout << socket.get_fd() << std::endl;
    if (!(socket.get_fd() < 0)) {
        // std::cout << msg << std::endl;
        if (socket.send(msg.c_str(), msg.size())) {
            return true;
        }
    }
    return false;
}
