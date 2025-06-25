
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
        std::optional<json> j = socket.recv();
        if (j.has_value()) {
            if (j.value()["identity"] == "GPGGA") {
                std::cout << j.value().dump(4) << std::endl;
                gga = GGA(j.value());
            }
            if (j.value()["identity"] == "NAV-ATT") {
                std::cout << j.value().dump(4) << std::endl;
                nav_att = Nav_Att(j.value());
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
