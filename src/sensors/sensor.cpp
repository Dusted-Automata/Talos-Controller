#include "sensor.hpp"

void
Sensor::start()
{
    if (running) return;
    running = true;

    if (sensor_thread.joinable()) {
        sensor_thread.join();
    }

    sensor_thread = std::thread(&Sensor::loop, this);
}

void
Sensor::stop()
{
    running = false;
    if (sensor_thread.joinable()) {
        sensor_thread.join();
    }
}
