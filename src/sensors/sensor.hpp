#pragma once
#include <atomic>
#include <optional>
#include <thread>

enum Sensor_Name { UBLOX_GGA, UBLOX_JSON };

template<typename T> struct Measurement {
    T val;
    Sensor_Name source;
};

template<typename T> class Sensor
{
  public:
    virtual ~Sensor() { stop(); };
    virtual void
    start()
    {
        if (running) return;
        running = true;

        if (sensor_thread.joinable()) {
            sensor_thread.join();
        }

        sensor_thread = std::thread(&Sensor::loop, this);
    };
    virtual void
    stop()
    {

        running = false;
        if (sensor_thread.joinable()) {
            sensor_thread.join();
        }
    }
    virtual void process() = 0;
    virtual void loop() = 0;
    // virtual std::optional<Measurement<>> poll() = 0;
  protected:
    std::atomic_bool running = false;
    std::thread sensor_thread;
    std::mutex measurement_mutex;
    std::optional<Measurement<T>> latest_measurement = {};
    Sensor_Name name;
};
