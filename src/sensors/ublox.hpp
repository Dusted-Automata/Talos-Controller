#pragma once
#include "NMEA.hpp"
#include "frames.hpp"
#include "sensor.hpp"
#include "socket.hpp"
#include <json.hpp>

using nlohmann::json;

struct Nav_Att {
    Nav_Att(json j)
    {
        accHeading = j["accHeading"];
        accPitch = j["accPitch"];
        accRoll = j["accRoll"];
        heading = j["heading"];
        pitch = j["pitch"];
        roll = j["roll"];
        iTOW = j["iTOW"];
        length = j["length"];
        msgmode = j["msgmode"];
    }

    double accHeading;
    double accPitch;
    double accRoll;
    double heading;
    double pitch;
    double roll;
    int32_t iTOW;
    int32_t length;
    int8_t msgmode;
};

struct Esf_Ins {
    int32_t iTOW;
    int32_t length;
    int8_t msgmode;
    int8_t version;
    double xAccel;
    double xAccelValid;
    double xAngRate;
    double xAngRateValid;
    double yAccel;
    double yAccelValid;
    double yAngRate;
    double yAngRateValid;
    double zAccel;
    double zAccelValid;
    double zAngRate;
    double zAngRateValid;
};

struct Ublox_Msgs {
    Nav_Att nav_att;
    GGA gga;
    // Esf_Ins esf_ins;
};

class Ublox : public Sensor<Ublox_Msgs>
{
    // Ublox_Msgs msg = {};

    std::optional<Nav_Att> nav_att;
    std::optional<GGA> gga;
    // std::optional<Esf_Ins> esf_ins;

  public:
    Ublox() {};
    TCP_Socket socket = TCP_Socket("127.0.0.1", 50020);
    Ring_Buffer<char, TCP_BUFFER_LENGTH * 2> buf;

    void loop() override;
    bool start() override;

    void consume(Msg_Type sensor);
    template<typename T> std::optional<T> get_latest(Msg_Type sensor);
};

void update_position(Ublox &ublox, Frames &frames);
void update_heading(Ublox &ublox, Frames &frames);
