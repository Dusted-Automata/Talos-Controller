#pragma once
#include <arpa/inet.h>
#include <array>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>

#include <json.hpp>

using nlohmann::json;

#define TCP_BUFFER_LENGTH 4096

class TCP_Socket
{
  private:
    struct sockaddr_in server;
    std::string server_ip;
    uint16_t port;
    std::array<char, TCP_BUFFER_LENGTH> recv_buf;
    int fd = -1;
    int buf_index = 0;

  public:
    TCP_Socket(std::string ip, uint16_t port) : server_ip(ip), port(port) {};
    ~TCP_Socket() { disconnect(); };
    bool connect();
    void disconnect();
    int get_fd();
    std::optional<json> recv();
};
