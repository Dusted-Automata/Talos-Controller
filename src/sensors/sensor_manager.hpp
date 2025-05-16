#pragma once
#include "sensor.hpp"
#include "ublox.hpp"
#include <netinet/in.h>
#include <thread>

class Sensor_Manager
{
    std::thread sensors_thread;
    std::optional<Measurement> latest_measurement = {};
    Ublox ublox;

  public:
    Sensor_Manager() {}
    ~Sensor_Manager() { shutdown(); }
    void readSensors();
    void init();
    void shutdown();
    void consume();
    void consume_measurement();
    std::optional<Measurement> get_latest(Sensor_Name sensor);
};
