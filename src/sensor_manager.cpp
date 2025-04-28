#include "sensor_manager.hpp"
#include <arpa/inet.h>
#include <unistd.h>

void
Sensor_Manager::loop()
{
    ublox.poll();
    while (!ublox.msgs.empty()) {
        latest_measurement.read = false;
        GGA gga = ublox.msgs.front();
        latest_measurement.ublox_measurement = std::move(gga);
        ublox.msgs.pop();
    }
}

void
Sensor_Manager::init()
{
}
