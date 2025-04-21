#pragma once
#include "types.hpp"
#include <arpa/inet.h>
#include <array>
#include <cstring>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <vector>

#define TCP_BUFFER_LENGTH 128

class Parser {
  public:
    virtual void process_data(std::vector<std::string> &msgs, std::array<char, TCP_BUFFER_LENGTH> &buf, size_t bytes_received) = 0;
};

class TCP_Socket
{
  private:
    int socket_fd;
    struct sockaddr_in server;
    std::string server_ip;
    int port;
    std::array<char, TCP_BUFFER_LENGTH> recv_buf;
    Parser &parser;

  public:
    TCP_Socket(std::string ip, int port, Parser &parser) : server_ip(ip), port(port), parser(parser) {};
    bool connect();
    bool recv(std::vector<std::string> &msgs);
    std::vector<std::string> recv_all();
    std::string ublox_Message();
};

class NMEA_Parser : public Parser
{
  private:
    Ring_Buffer<char> ring = Ring_Buffer<char>(TCP_BUFFER_LENGTH*2);

    inline bool findStart(Ring_Buffer<char> &buf, size_t &index);
    inline bool findEnd(Ring_Buffer<char> &buf, size_t &index);

  public:
    void process_data(std::vector<std::string> &msgs, std::array<char, TCP_BUFFER_LENGTH> &buf, size_t bytes_received) override;
};

