#pragma once
#include "sensor.hpp"
#include "ublox.hpp"
#include <netinet/in.h>
#include <thread>

struct Latest_Measurement {
    std::optional<Measurement<GGA>> ublox_GGA;
    std::optional<Measurement<json>> ublox_json;
};

class Sensor_Manager
{
    std::thread sensors_thread;
    Latest_Measurement latest_measurement = {};
    Ublox_GGA ublox_gga;
    Ublox_JSON ublox_json;

  public:
    Sensor_Manager() {}
    ~Sensor_Manager() { shutdown(); }
    void readSensors();
    void init();
    void shutdown();
    void consume();
    void consume_measurement(Sensor_Name sensor);
    template<typename T> std::optional<Measurement<T>> get_latest(Sensor_Name sensor);
};
