#include "sensor_manager.hpp"
#include "ublox.hpp"
#include <arpa/inet.h>
#include <poll.h>
#include <unistd.h>

// void
// Sensor_Manager::loop()
// {
//     constexpr int POLL_TIMEOUT_MS = 500;
//
//     while (running) {
//         pollfd pfd;
//         pfd.fd = ublox.socket.socket_fd;
//         pfd.events = POLLIN;
//
//         int ret = poll(&pfd, 1, POLL_TIMEOUT_MS);
//
//         if (ret > 0) {
//             if (pfd.revents & POLLIN) {
//                 if (ublox.read()) {
//                     Measurement measurement;
//                     while (!ublox.msgs.empty()) {
//                         GGA gga = ublox.msgs.front();
//                         measurement = { .ublox_measurement = gga };
//                         ublox.msgs.pop();
//                     }
//                     std::unique_lock<std::mutex> lock(sensor_mutex);
//                     latest_measurement = measurement;
//                 }
//             }
//             if (pfd.revents & (POLLERR | POLLHUP)) {
//                 std::cerr << "Socket error or hang-up" << std::endl;
//                 break;
//             }
//         } else if (ret < 0) {
//             if (errno == EINTR) continue;
//             std::cerr << "poll error: " << strerror(errno) << std::endl;
//             break;
//         }
//     }
// }

void
Sensor_Manager::consume_and_produce()
{
    ublox.process();

    while (!ublox.msgs.empty()) {
        GGA gga = ublox.msgs.front();
        latest_measurement = { .ublox_measurement = gga, .source = Sensor_Name::UBLOX };
        ublox.msgs.pop();
    }
}

void
Sensor_Manager::init()
{

    ublox.start();
}

void
Sensor_Manager::shutdown()
{
    ublox.stop();
}

void
Sensor_Manager::consume_measurement()
{
    latest_measurement.reset();
}

std::optional<Measurement>
Sensor_Manager::get_latest(Sensor_Name sensor)
{
    // std::unique_lock<std::mutex> lock(sensor_mutex);
    switch (sensor) {
    case Sensor_Name::UBLOX: return latest_measurement;
    }
    return std::nullopt;
}
