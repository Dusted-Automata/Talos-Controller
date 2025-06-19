#pragma once
#include <atomic>
#include <optional>
#include <thread>

enum Msg_Type { NAV_ATT, ESF_INS };

template<typename T> class Sensor
{
  public:
    virtual ~Sensor() { stop(); };
    virtual bool
    start()
    {
        if (running) return true;

        if (sensor_thread.joinable()) {
            sensor_thread.join();
        }

        sensor_thread = std::thread(&Sensor::loop, this);
        running = true;
        return true;
    };
    virtual void
    stop()
    {

        running = false;
        if (sensor_thread.joinable()) {
            sensor_thread.join();
        }
    }
    virtual void loop() = 0;
    // virtual std::optional<Measurement<>> poll() = 0;
  protected:
    std::atomic_bool running = false;
    std::thread sensor_thread;
    std::mutex measurement_mutex;
    std::optional<T> latest_measurement = {};
};
