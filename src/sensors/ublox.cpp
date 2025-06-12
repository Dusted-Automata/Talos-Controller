
#include "ublox.hpp"
#include <arpa/inet.h>
#include <array>
#include <cstdint>
#include <cstring>
#include <netinet/in.h>
#include <sstream>
#include <string>
#include <unistd.h>

void
parse_time(GGA &gga, const std::string &time)
{
    gga.time.hh = static_cast<uint8_t>(std::stoul(time.substr(0, 2)));
    gga.time.mm = static_cast<uint8_t>(std::stoul(time.substr(2, 2)));
    gga.time.ss = static_cast<uint8_t>(std::stoul(time.substr(4, 2)));
    gga.time.ms = static_cast<uint16_t>(std::stoul(time.substr(7)));
}
void
parse_latlng(
    GGA &gga, const std::string &lat, const std::string lat_dir, const std::string lng, const std::string lng_dir)
{

    // Check that both fields exist.
    if (!lat.empty() && !lat_dir.empty()) {
        double latitude = std::stod(lat.substr(0, 2));
        latitude += std::stod(lat.substr(2)) / 60.0;
        if (lat_dir == "S") {
            latitude *= -1.0;
        }

        gga.latlng.lat = latitude;
    }

    if (!lng.empty() && !lng_dir.empty()) {
        double longitude = std::stod(lng.substr(0, 3));
        longitude += std::stod(lng.substr(3)) / 60.0;
        if (lng_dir == "S") {
            longitude *= -1.0;
        }

        gga.latlng.lng = longitude;
    }
}

void
parse_fix(GGA &gga, const std::string &fix)
{
    if (fix.empty()) {
        return;
    }
    gga.fix = static_cast<GGA::Fix>(std::stoi(fix));
}

uint8_t
parse_uint8(const std::string &field)
{
    if (field.empty()) {
        return 0;
    }
    return static_cast<uint8_t>(std::stoul(field));
};

float
parse_float(const std::string &field)
{
    if (field.empty()) {
        return 0.0;
    }
    return std::stof(field);
}

double
parse_double(const std::string &field)
{
    if (field.empty()) {
        return 0.0;
    }
    return std::stod(field);
}

GGA
parse_gga(std::string &msg)
{
    msg.replace(msg.size() - 5, 5, "\0"); // remove return and line feed and checksum;
    std::stringstream ss(msg);
    const size_t SIZE = 17;
    std::array<std::string, SIZE> arr;
    std::string token;

    for (size_t i = 0; i < SIZE; i++) {
        if (std::getline(ss, token, ',')) {
            arr[i] = token;
        }
    }

    GGA gga = {};
    parse_time(gga, arr[1]);
    parse_latlng(gga, arr[2], arr[3], arr[4], arr[5]);
    parse_fix(gga, arr[6]);

    gga.num_satalites = parse_uint8(arr[7]);
    gga.hddp = parse_float(arr[8]);
    gga.alt = parse_double(arr[9]);
    gga.geoid_seperation = parse_float(arr[11]);
    gga.diff_age = parse_float(arr[13]);
    gga.diff_station = parse_float(arr[14]);

    return gga;
}

NMEA_Cmd
to_nmea_cmd(const std::string &key)
{
    if (key == "GPGGA") return NMEA_Cmd::GGA;
    return NMEA_Cmd::UNKNOWN;
}

NMEA_Cmd
extract_command(const std::string &cmd_str)
{
    size_t startPos = 1;
    size_t endPos = cmd_str.find(',', startPos);
    if (endPos == std::string::npos) {
        return NMEA_Cmd::UNKNOWN;
    }

    auto cmd = cmd_str.substr(startPos, endPos - 1);
    return to_nmea_cmd(cmd);
}

void
Ublox_GGA::process()
{
    while (!buf.empty()) {
        std::string i = buf.front();
        NMEA_Cmd cmd = extract_command(i);
        switch (cmd) {
        case NMEA_Cmd::UNKNOWN:
            // std::cout << "Unknown command" << std::endl;
            break;
        case NMEA_Cmd::GGA:
            GGA gga = parse_gga(i);
            // gga.print();
            msgs.push_back(gga);
            break;
        }
        buf.pop();
    }
}

void
Ublox_GGA::loop()
{

    while (socket.recv(buf) && running) {
        process();
    }
}

void
Ublox_GGA::start()
{

    if (!socket.connect()) {
        std::cerr << "Ublx couldn't connect to nmea_socket" << std::endl;
        return;
    };

    if (socket.fd < 0) {
        return;
    }

    if (running) return;
    running = true;

    if (sensor_thread.joinable()) {
        sensor_thread.join();
    }

    sensor_thread = std::thread(&Sensor::loop, this);
}

void
Ublox_simple::process()
{
    while (!buf.empty()) {
        json msg = json::parse(buf.front());
        // std::string identity": "NAV-ATT",
        // std::cout << msg.dump(4) << std::endl;
        if (msg["identity"] == "ESF-INS") {
            std::cout << msg.dump(4) << std::endl;
            // msgs.push_back(std::move(msg));
        }
        if (msg["identity"] == "NAV-ATT") {
            // std::cout << msg.dump(4) << std::endl;
            msgs.push_back(std::move(msg));
        }
        buf.pop();
    }
}

void
Ublox_simple::loop()
{

    while (socket.recv(buf) && running) {
        process();
    }
}

void
Ublox_simple::start()
{

    if (!socket.connect()) {
        std::cerr << "Ublx couldn't connect to json_socket" << std::endl;
        return;
    };

    if (socket.fd < 0) {
        return;
    }

    if (running) return;
    running = true;

    if (sensor_thread.joinable()) {
        sensor_thread.join();
    }

    sensor_thread = std::thread(&Sensor::loop, this);
}
