#pragma once
#include "types.hpp"
#include <arpa/inet.h>
#include <array>
#include <queue>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <vector>

#define TCP_BUFFER_LENGTH 4096

class Socket_Parser
{
  public:
    virtual void push(std::queue<std::string> &msgs, std::span<const char> data) = 0;
};

class TCP_Socket
{
  private:
    struct sockaddr_in server;
    std::string server_ip;
    uint16_t port;
    std::array<char, TCP_BUFFER_LENGTH> recv_buf;
    Socket_Parser &parser;

  public:
    TCP_Socket(std::string ip, uint16_t port, Socket_Parser &parser) : server_ip(ip), port(port), parser(parser) {};
    ~TCP_Socket() { disconnect(); };
    bool connect();
    void disconnect();
    bool recv(std::queue<std::string> &msgs);
    std::vector<std::string> recv_all();
    std::string ublox_Message();
    int socket_fd = -1;
};

class NMEA_Parser : public Socket_Parser
{
  private:
    Ring_Buffer<char, TCP_BUFFER_LENGTH * 2> ring;

    inline bool skip_UBX();
    bool extract_NMEA(std::string &out);
    inline bool verify_Checksum(std::string_view s);
    inline uint8_t hex_Val(char c);

  public:
    void push(std::queue<std::string> &msgs, std::span<const char> data) override;
};

class Ublox_JSON_Parser : public Socket_Parser
{

  private:
    std::vector<char> buf;
    Ring_Buffer<char, TCP_BUFFER_LENGTH * 4> ring;

  public:
    void push(std::queue<std::string> &msgs, std::span<const char> data) override;
};
