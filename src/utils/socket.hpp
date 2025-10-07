#pragma once
#include "types.hpp"
#include <arpa/inet.h>
#include <array>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <poll.h>

#include <json.hpp>

using nlohmann::json;

#define TCP_BUFFER_LENGTH 4096
#define TCP_BUFFER_LENGTH_CLIENT 256

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

struct Client {
    int fd;

    struct sockaddr_in peer;
    socklen_t peer_len;

    i32 in_len;
    i32 out_offset;
    i32 out_len;

    Ring_Buffer<char, TCP_BUFFER_LENGTH_CLIENT> buf;
};

struct TCP_Server {
    sockaddr_in address;
    uint16_t port = 55550;
    int fd;
    int opt = 1;
    int addrlen = sizeof(address);

    static const i32 MAX_FDS=16;
    nfds_t nfds = 0;
    pollfd fds[MAX_FDS];
    Client clients[MAX_FDS-1];

    std::atomic_bool running = false;
    std::array<char, TCP_BUFFER_LENGTH> recv_buf;
};

bool tcp_server_setup(TCP_Server &server);
bool socket_recv(int fd, std::array<char, TCP_BUFFER_LENGTH> &recv_buf, Ring_Buffer<char, TCP_BUFFER_LENGTH * 2> &ring);
bool tcp_send(const int fd, const char *buf, size_t length);
void tcp_server_loop(TCP_Server &server);
