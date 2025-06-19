
#include "ublox.hpp"
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
    case Msg_Type::ESF_INS: esf_ins.reset(); return;
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
std::optional<Esf_Ins>
Ublox::get_latest(Msg_Type msg)
{
    // std::unique_lock<std::mutex> lock(sensor_mutex);
    if (msg == ESF_INS) {
        return esf_ins;
    }
    return std::nullopt;
}
