#include "sensor_manager.hpp"
#include <arpa/inet.h>
#include <unistd.h>

void
Sensor_Manager::loop()
{
    while (running) {
        ublox.poll();
        while (!ublox.msgs.empty()) {
            latest_measurement.read = false;
            GGA gga = ublox.msgs.front();
            latest_measurement.ublox_measurement = std::move(gga);
            ublox.msgs.pop();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

void
Sensor_Manager::init()
{
    if (running) return;
    running = true;

    if (sensors_thread.joinable()) {
        sensors_thread.join();
    }

    sensors_thread = std::thread(&Sensor_Manager::loop, this);
}

void
Sensor_Manager::shutdown()
{
    running = false;
    if (sensors_thread.joinable()) {
        sensors_thread.join();
    }
}
