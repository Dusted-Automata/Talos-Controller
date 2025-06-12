#pragma once
#include "types.hpp"
#include <queue>
#include <string>
#include <vector>

class Socket_Parser
{
  public:
    virtual void push(std::queue<std::string> &msgs, std::span<const char> data) = 0;
};

class NMEA_Parser : public Socket_Parser
{
  private:
    Ring_Buffer<char, 8192> ring;

    inline bool skip_UBX();
    bool extract_NMEA(std::string &out);
    inline bool verify_Checksum(std::string_view s);
    inline uint8_t hex_Val(char c);

  public:
    void push(std::queue<std::string> &msgs, std::span<const char> data) override;
};

class JSON_Parser : public Socket_Parser
{

  private:
    std::vector<char> buf;
    Ring_Buffer<char, 16384> ring;

  public:
    void push(std::queue<std::string> &msgs, std::span<const char> data) override;
};
