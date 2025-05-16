#include "sensor_manager.hpp"
#include "ublox.hpp"
#include <arpa/inet.h>
#include <poll.h>
#include <unistd.h>

void
Sensor_Manager::consume()
{
    while (!ublox.msgs.empty()) {
        GGA gga = ublox.msgs.back();
        latest_measurement = { .ublox = gga, .source = Sensor_Name::UBLOX };
        ublox.msgs.clear(); // Currently we are only interested in the latest measurement. I might switch it to just an
                            // optional or something
    }
}

void
Sensor_Manager::init()
{

    // std::thread ublox_startup_thread(&Ublox::start, ublox);
    // ublox_startup_thread.join();
    ublox.start();
}

void
Sensor_Manager::shutdown()
{
    ublox.stop();
}

void
Sensor_Manager::consume_measurement()
{
    latest_measurement.reset();
}

std::optional<Measurement>
Sensor_Manager::get_latest(Sensor_Name sensor)
{
    // std::unique_lock<std::mutex> lock(sensor_mutex);
    switch (sensor) {
    case Sensor_Name::UBLOX: return latest_measurement;
    }
    return std::nullopt;
}
