#include "sensor_manager.hpp"
#include "ublox.hpp"
#include <arpa/inet.h>
#include <poll.h>
#include <unistd.h>

void
Sensor_Manager::consume()
{
    while (!ublox_gga.msgs.empty()) {
        GGA gga = ublox_gga.msgs.back();
        latest_measurement.ublox_GGA = { .val = gga, .source = Sensor_Name::UBLOX_GGA };
        ublox_gga.msgs.clear(); // Currently we are only interested in the latest measurement. I might switch it to
                                // just an optional or something
    }

    while (!ublox_json.msgs.empty()) {
        json j = ublox_json.msgs.back();
        NAV_ATT nav(j);
        latest_measurement.ublox_simple = {
            .val = { .nav_att = j, .esf_ins = {} },
            .source = Sensor_Name::UBLOX_SIMPLE
        };
        ublox_json.msgs.clear();
    }
}

void
Sensor_Manager::init()
{

    ublox_gga.start();
    ublox_json.start();
}

void
Sensor_Manager::shutdown()
{
    ublox_gga.stop();
    ublox_json.stop();
}

void
Sensor_Manager::consume_measurement(Sensor_Name sensor)
{
    switch (sensor) {
    case Sensor_Name::UBLOX_GGA: latest_measurement.ublox_GGA.reset(); return;
    case Sensor_Name::UBLOX_SIMPLE: latest_measurement.ublox_simple.reset(); return;
    }
}

template<>
std::optional<Measurement<GGA>>
Sensor_Manager::get_latest(Sensor_Name sensor)
{
    // std::unique_lock<std::mutex> lock(sensor_mutex);
    if (sensor == UBLOX_GGA) {
        return latest_measurement.ublox_GGA;
    }
    return std::nullopt;
}

template<>
std::optional<Measurement<Simple_Ublox>>
Sensor_Manager::get_latest(Sensor_Name sensor)
{
    // std::unique_lock<std::mutex> lock(sensor_mutex);
    if (sensor == UBLOX_SIMPLE) {
        return latest_measurement.ublox_simple;
    }
    return std::nullopt;
}
