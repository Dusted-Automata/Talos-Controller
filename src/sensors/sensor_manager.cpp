#include "sensor_manager.hpp"
#include "ublox.hpp"
#include <arpa/inet.h>
#include <poll.h>
#include <unistd.h>

void
Sensor_Manager::recv_latest()
{
    if (ublox.nav_att.has_value()) {
        latest_measurement.nav_att.swap(ublox.nav_att);
    }
    if (ublox.esf_ins.has_value()) {
        latest_measurement.esf_ins.swap(ublox.esf_ins);
    }
}

void
Sensor_Manager::init()
{

    ublox.start();
}

void
Sensor_Manager::shutdown()
{
    ublox.stop();
}

void
Sensor_Manager::consume(Msg_Type msg)
{
    switch (msg) {
    case Msg_Type::NAV_ATT: latest_measurement.nav_att.reset(); return;
    case Msg_Type::ESF_INS: latest_measurement.esf_ins.reset(); return;
    }
}

template<>
std::optional<Nav_Att>
Sensor_Manager::get_latest(Msg_Type msg)
{
    // std::unique_lock<std::mutex> lock(sensor_mutex);
    if (msg == NAV_ATT) {
        return latest_measurement.nav_att;
    }
    return std::nullopt;
}

template<>
std::optional<Esf_Ins>
Sensor_Manager::get_latest(Msg_Type msg)
{
    // std::unique_lock<std::mutex> lock(sensor_mutex);
    if (msg == ESF_INS) {
        return latest_measurement.esf_ins;
    }
    return std::nullopt;
}
