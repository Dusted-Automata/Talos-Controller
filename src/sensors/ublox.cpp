
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
                        // std::cout << j.dump(4) << std::endl;
                        gga = GGA(j);
                    }
                    if (j["identity"] == "NAV-ATT") {
                        // std::cout << j.dump(4) << std::endl;
                        nav_att = Nav_Att(j);
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
