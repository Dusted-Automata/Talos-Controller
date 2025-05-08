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
                    while (!ublox.msgs.empty()) {
                        latest_measurement.read = false;
                        GGA gga = std::move(ublox.msgs.front());
                        latest_measurement.ublox_measurement = std::move(gga);
                        ublox.msgs.pop();
                    }
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
