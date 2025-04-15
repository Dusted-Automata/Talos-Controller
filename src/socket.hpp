#pragma once
#include <arpa/inet.h>
#include <array>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <vector>

#define TCP_BUFFER_LENGTH 128

struct Half_Message
{
    char buf[TCP_BUFFER_LENGTH];
    size_t end;
    bool exists;
};

struct Stream_Buffers
{
    char recv[TCP_BUFFER_LENGTH];
    Half_Message half_message;
};

class TCP_Socket
{
  private:
    int socket_fd;
    struct sockaddr_in server;
    std::string server_ip;
    int port;
    Stream_Buffers stream_buffers;

  public:
    TCP_Socket(std::string ip, int port) : server_ip(ip), port(port){};
    bool connect();
    bool recv(std::vector<std::string> &msgs);
    std::vector<std::string> recv_all();
    std::string ublox_Message();
};
