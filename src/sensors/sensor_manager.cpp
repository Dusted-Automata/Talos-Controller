#include "sensor_manager.hpp"
#include <arpa/inet.h>
#include <poll.h>
#include <unistd.h>

void
Sensor_Manager::loop()
{
    constexpr int POLL_TIMEOUT_MS = 500;

    while (running) {
        pollfd pfd;
        pfd.fd = ublox.socket.socket_fd;
        pfd.events = POLLIN;

        int ret = poll(&pfd, 1, POLL_TIMEOUT_MS);

        if (ret > 0) {
            if (pfd.revents & POLLIN) {
                if (ublox.read()) {
                    Measurement measurement;
                    while (!ublox.msgs.empty()) {
                        GGA gga = ublox.msgs.front();
                        measurement = { .ublox_measurement = gga };
                        ublox.msgs.pop();
                    }
                    std::unique_lock<std::mutex> lock(sensor_mutex);
                    latest_measurement = measurement;
                }
            }
            if (pfd.revents & (POLLERR | POLLHUP)) {
                std::cerr << "Socket error or hang-up" << std::endl;
                break;
            }
        } else if (ret < 0) {
            if (errno == EINTR) continue;
            std::cerr << "poll error: " << strerror(errno) << std::endl;
            break;
        }
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

// if (latest_measurement.has_value()) {
//     GGA gga = std::move(latest_measurement->ublox_measurement);
//     latest_measurement.reset(); // equivalent to setting read = true or marking as processed
// }
std::optional<Measurement>
Sensor_Manager::get_latest()
{
    std::unique_lock<std::mutex> lock(sensor_mutex);
    return latest_measurement;
}
