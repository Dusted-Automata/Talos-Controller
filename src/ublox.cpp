
#include "ublox.hpp"
#include <arpa/inet.h>
#include <cstdint>
#include <cstring>
#include <netinet/in.h>
#include <sstream>
#include <string>
#include <unistd.h>

void parse_time(GGA &gga, std::string &time)
{
    std::cout << time << std::endl;
    std::cout << time.substr(2, 2) << std::endl;
    std::cout << (int)static_cast<uint8_t>(std::stol(time.substr(2, 2))) << std::endl;

    gga.time.hh = static_cast<uint8_t>(std::stoul(time.substr(0, 2)));
    gga.time.mm = static_cast<uint8_t>(std::stoul(time.substr(2, 2)));
    gga.time.ss = static_cast<uint8_t>(std::stoul(time.substr(4, 2)));
    gga.time.ms = static_cast<uint16_t>(std::stoul(time.substr(7)));
}
void parse_latlng(GGA &gga, std::string &lat, std::string lat_dir, std::string lng,
                  std::string lng_dir)
{

    // Check that both fields exist.
    if (!lat.empty() && !lat_dir.empty())
    {
        double latitude = std::stod(lat.substr(0, 2));
        latitude += std::stod(lat.substr(2)) / 60.0;
        if (lat_dir == "S")
        {
            latitude *= -1.0;
        }

        gga.latlng.lat = latitude;
    }

    if (!lng.empty() && !lng_dir.empty())
    {
        double longitude = std::stod(lng.substr(0, 3));
        std::cout << longitude << std::endl;
        longitude += std::stod(lng.substr(3)) / 60.0;
        if (lng_dir == "S")
        {
            longitude *= -1.0;
        }

        gga.latlng.lng = longitude;
    }
}

void parse_fix(GGA &gga, std::string &fix)
{
    if (fix.empty())
    {
        return;
    }
    gga.fix = static_cast<GGA::Fix>(std::stoi(fix));
}

uint8_t parse_uint8(std::string &field)
{
    if (field.empty())
    {
        return 0;
    }
    return static_cast<uint8_t>(std::stoul(field));
};

float parse_float(std::string &field)
{
    if (field.empty())
    {
        return 0.0;
    }
    return static_cast<uint8_t>(std::stof(field));
}

void parse_hddp(GGA &gga, std::string &hddp);
void parse_alt(GGA &gga, std::string &alt);
void parse_geoid_seperation(GGA &gga, std::string &seperation);
void parse_diff_age(GGA &gga, std::string &age);
void parse_diff_station(GGA &gga, std::string &station);

GGA parse_gga(const std::string &msg)
{

    std::stringstream ss(msg);
    const size_t SIZE = 15;
    std::array<std::string, SIZE> arr;
    std::string token;

    for (int i = 0; i < SIZE; i++)
    {
        if (std::getline(ss, token, ','))
        {
            arr[i] = token;
        }
    }

    arr[SIZE - 1] = arr[SIZE - 1].substr(0, arr[SIZE - 1].find('*')); // removing checksum

    GGA gga = {};
    parse_time(gga, arr[1]);
    parse_latlng(gga, arr[2], arr[3], arr[4], arr[5]);
    parse_fix(gga, arr[6]);

    gga.num_satalites = parse_uint8(arr[6]);
    gga.hddp = parse_float(arr[8]);
    gga.alt = parse_float(arr[9]);
    gga.geoid_seperation = parse_float(arr[11]);
    gga.diff_age = parse_float(arr[13]);
    gga.diff_station = parse_float(arr[14]);

    return gga;
}

bool Ublox::poll()
{
    std::vector<std::string> msgs = tcp.recv_all();
    for (auto &i : msgs)
    {
        GGA gga = parse_gga(i);
        gga.print();
    }

    return true;
}

GGA Ublox::read() { return GGA{}; };
