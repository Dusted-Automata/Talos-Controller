#pragma once
#include "types.hpp"
#include <arpa/inet.h>
#include <array>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>

#include <json.hpp>

using nlohmann::json;

#define TCP_BUFFER_LENGTH 4096

class TCP_Client
{
  private:
    struct sockaddr_in server;
    std::string server_ip;
    uint16_t port;
    std::array<char, TCP_BUFFER_LENGTH> recv_buf;
    int fd = -1;
    ssize_t buf_index = 0;

  public:
    TCP_Client(std::string ip, uint16_t port) : server_ip(ip), port(port) {};
    ~TCP_Client() { disconnect(); };
    bool connect();
    void disconnect();
    int get_fd();
    std::optional<json> recv();

    bool recv(Ring_Buffer<char, TCP_BUFFER_LENGTH * 2> &ring);
    bool send(const char *buf, size_t size);
};

struct TCP_Server {
    struct sockaddr_in address;
    uint16_t port;
    int fd;
    int client_socket;
    int opt = 1;
    int addrlen = sizeof(address);
};

bool tcp_server_setup(TCP_Server &server);
void tcp_server_handle_client(TCP_Server &server);
void tcp_server_start(TCP_Server &server);
bool socket_recv(int fd, std::array<char, TCP_BUFFER_LENGTH> recv_buf, Ring_Buffer<char, TCP_BUFFER_LENGTH * 2> &ring);
bool tcp_send(const int fd, const char *buf, size_t length);
