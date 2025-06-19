#pragma once
#include "sensor.hpp"
#include "ublox.hpp"
#include <netinet/in.h>
#include <thread>

struct Latest_Measurement {
    std::optional<Nav_Att> nav_att;
    std::optional<Esf_Ins> esf_ins;
};

class Sensor_Manager
{
    std::thread sensors_thread;
    Latest_Measurement latest_measurement = {};
    Ublox ublox;

  public:
    Sensor_Manager() {}
    ~Sensor_Manager() { shutdown(); }
    void readSensors();
    void init();
    void shutdown();
    void recv_latest();
    void consume(Msg_Type sensor);
    template<typename T> std::optional<T> get_latest(Msg_Type sensor);
};
