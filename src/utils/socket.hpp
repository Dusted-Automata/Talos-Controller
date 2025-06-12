#pragma once
#include "socket_parser.hpp"
#include <arpa/inet.h>
#include <array>
#include <queue>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <vector>

#define TCP_BUFFER_LENGTH 4096

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
