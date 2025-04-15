#pragma once
#include <arpa/inet.h>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>

class TCP_Socket
{
  private:
    int socket_fd;
    struct sockaddr_in server;
    std::string server_ip;
    int port;

  public:
    TCP_Socket(std::string ip, int port) : server_ip(ip), port(port){};
    bool connect();
    bool listen(char *buffer, size_t buffer_size);
};
