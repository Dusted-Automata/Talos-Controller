#include "socket.hpp"
#include <fcntl.h>
#include <iostream>
#include <unistd.h>

bool
TCP_Client::connect()
{
    disconnect();
    fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd == -1) {
        std::cerr << "Could not create socket" << std::endl;
        return false;
    }

    server.sin_family = AF_INET;
    server.sin_addr.s_addr = inet_addr(server_ip.c_str());
    server.sin_port = htons(port);

    int con = ::connect(fd, (struct sockaddr *)&server, sizeof(server));

    if (con < 0) {
        std::cerr << "Connection failed to " << server_ip << ":" << port << std::endl;
        disconnect();
        return false;
    }
    std::cout << "Connected to " << server_ip << ":" << port << std::endl;

    return true;
}

bool
socket_recv(int fd, std::array<char, TCP_BUFFER_LENGTH> &recv_buf, Ring_Buffer<char, TCP_BUFFER_LENGTH * 2> &ring)
{
    if (fd == -1) {
        std::cerr << "Socket is closed" << std::endl;
        return false;
    }

    ssize_t bytes_received = ::recv(fd, recv_buf.data(), recv_buf.size(), 0);
    size_t wrote = ring.write(std::span(recv_buf.data(), bytes_received));
    if (wrote != static_cast<size_t>(bytes_received)) {
        // policy: drop oldest, signal backpressure, or mark Error
        std::cerr << "ring buffer overflow (wrote " << wrote << " of " << bytes_received << ")\n";
    }

    if (bytes_received == 0) {
        std::cerr << "socket closed" << std::endl;
        ::close(fd);
        return false;
    }
    if (bytes_received < 0) {
        std::cerr << "recv failed" << std::endl;
        ::close(fd);
        return false;
    }

    return true;
}

bool
socket_recv_non_blocking(int fd, std::array<char, TCP_BUFFER_LENGTH> &recv_buf, Ring_Buffer<char, TCP_BUFFER_LENGTH_CLIENT> &ring)
{
    if (fd == -1) {
        std::cerr << "Socket is closed" << std::endl;
        return false;
    }

    for (;;) {
        ssize_t bytes_received = ::recv(fd, recv_buf.data(), recv_buf.size(), 0);
        if (bytes_received > 0) {
            std::cout.write(recv_buf.data(), bytes_received);
            size_t wrote = ring.write(std::span(recv_buf.data(), bytes_received));
            if (wrote != static_cast<size_t>(bytes_received)) {
                // policy: drop oldest, signal backpressure, or mark Error
                std::cerr << "ring buffer overflow (wrote " << wrote << " of " << bytes_received << ")\n";
                ::close(fd);
                return false;
            }
            continue;
        }
        if (bytes_received == 0) {
            std::cerr << "socket closed" << std::endl;
            ::close(fd);
            return false;
        }

        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return true;
        }
        if (errno == EINTR) {
            continue;
        }

        std::cerr << "recv failed" << std::endl;
        ::close(fd);
        return false;
    }

    return true;
}

static int set_nonblocking(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags == -1) return -1;
    if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) == -1) return -1;
    return 0;
}

void
add_fd(int fd, TCP_Server &server) {
    server.fds[server.nfds].fd = fd;
    server.fds[server.nfds].events = POLLIN; // monitor for readable
    server.fds[server.nfds].revents = 0;
    server.nfds++;
}

void
remove_fd(int id, TCP_Server &server) {
    server.fds[id] = server.fds[server.nfds - 1];
    server.nfds--;
}

void
remove_fd_with_client(int id, TCP_Server &server) {
    server.fds[id] = server.fds[server.nfds - 1];
    server.clients[id] = server.clients[server.nfds - 1];
    server.nfds--;
}

bool
tcp_server_setup(TCP_Server &server)
{
    // Create socket file descriptor
    server.fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server.fd == 0) {
        perror("socket failed");
        return false;
    }

    // Always okay on Linux/BSD/macOS:
    if (setsockopt(server.fd, SOL_SOCKET, SO_REUSEADDR, &server.opt, sizeof(server.opt)) < 0) {
        perror("setsockopt(SO_REUSEADDR)");
        return false;
    }

    server.address.sin_family = AF_INET;
    server.address.sin_addr.s_addr = INADDR_ANY; //does this need htonl?
    server.address.sin_port = htons(server.port);

    // Bind the socket to the network address and port
    if (bind(server.fd, (sockaddr*)&server.address, sizeof(server.address)) < 0) {
        perror("bind failed");
        return false;
    }

    std::cout << "Server initialized on port " << server.port << std::endl;

    if (listen(server.fd, SOMAXCONN) < 0) {
        perror("socket listen failed");
        return false;
    }

    if (set_nonblocking(server.fd) < 0) {
        perror("listen");
        return false;
    }

    add_fd(server.fd, server);
    std::cout << "Server listening for connections..." << std::endl;

    // Accept a connection
    // int client_socket = -1;
    // if ((client_socket = accept(server.fd, (struct sockaddr *)&server.address, (socklen_t *)&server.addrlen))
    //     < 0) {
    //     server.clients.push_back(client_socket);
    //     perror("accept");
    //     return false;
    // }

    std::cout << "Connection accepted from " << inet_ntoa(server.address.sin_addr) << std::endl;
    return true;
}


void
tcp_server_loop(TCP_Server &server) {
    while (server.running) {
        int nready = poll(server.fds, server.nfds, -1); // block until something happens
        if (nready < 0) {
            if (errno == EINTR) continue; // interrupted by signal
            perror("TCP_Server poll failed");
        }

        // 1) New inbound connections?
        if (server.fds[0].revents & POLLIN) {
            --nready;
            // Accept until we run out (non-blocking)
            while (1) {
                sockaddr_in client;
                socklen_t len = sizeof(client);
                int client_fd = accept(server.fd, (sockaddr *)&client, &len);
                if (client_fd < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) break;
                    perror("accept");
                    break;
                }
                if (set_nonblocking(client_fd) < 0) {
                    perror("set_nonblocking(client)");
                    close(client_fd);
                    continue;
                }
                if (server.nfds >= server.MAX_FDS) {
                    fprintf(stderr, "Too many clients; rejecting.\n");
                    close(client_fd);
                    continue;
                }
                char ip[64];
                inet_ntop(AF_INET, &client.sin_addr, ip, sizeof(ip));
                printf("New connection fd=%d from %s:%d\n",
                       client_fd, ip, ntohs(client.sin_port));

                add_fd(client_fd, server);
                server.clients[server.nfds].fd = client_fd;
                server.clients[server.nfds].peer = client;
                server.clients[server.nfds].peer_len = len;
            }
        }

        // 2) Data from clients / hangups / errors
        for (nfds_t i = 1; i < server.nfds; ++i) {
            if (nready <= 0) break;
            short return_events = server.fds[i].revents;
            if (!return_events) continue;
            --nready;

            int client_fd = server.fds[i].fd;

            if (return_events & (POLLHUP | POLLERR | POLLNVAL)) {
                // Peer closed or error
                printf("Closing fd=%d (event=0x%x)\n", client_fd, return_events);
                close(client_fd);
                // Remove this entry by swapping with last
                remove_fd_with_client(i, server);
                i--; 
                continue;
            }

            if (return_events & POLLIN) {
                // Read all available data (edge-friendly on non-blocking)
                bool success = socket_recv_non_blocking(client_fd, server.recv_buf, server.clients[i].buf);
                if (!success) {
                    remove_fd_with_client(i, server);
                    --i; 
                }
            }
        }
    }
}



int
TCP_Client::get_fd()
{
    return fd;
}

void
TCP_Client::disconnect()
{
    if (fd != -1) {
        ::close(fd);
        fd = -1;
    }
}

bool
TCP_Client::recv(Ring_Buffer<char, TCP_BUFFER_LENGTH * 2> &ring)
{

    if (fd == -1) {
        std::cerr << "Socket is closed" << std::endl;
        return false;
    }

    ssize_t bytes_received;

    bytes_received = ::recv(fd, recv_buf.data(), recv_buf.size(), 0);
    ring.write(std::span(recv_buf.data(), bytes_received));

    if (bytes_received == 0) {
        std::cerr << "socket closed" << std::endl;
        disconnect();
        return false;
    }
    if (bytes_received < 0) {
        std::cerr << "recv failed" << std::endl;
        disconnect();
        return false;
    }

    return true;
}

bool
TCP_Client::send(const char *buf, size_t length)
{
    size_t total_sent = 0;
    while (total_sent < length) {
        ssize_t bytes_sent = ::send(fd, buf + total_sent, length - total_sent, 0);
        if (bytes_sent <= 0) {
            if (errno == EINTR) continue; // Retry if interrupted
            std::cerr << "Send error: " << strerror(errno) << std::endl;
            return false;
        }
        total_sent += bytes_sent;
    }
    return false;
}

bool
tcp_send(const int fd, const char *buf, size_t length)
{
    size_t total_sent = 0;
    while (total_sent < length) {
        ssize_t bytes_sent = ::send(fd, buf + total_sent, length - total_sent, 0);
        if (bytes_sent <= 0) {
            if (errno == EINTR) continue; // Retry if interrupted
            std::cerr << "Send error: " << strerror(errno) << std::endl;
            return false;
        }
        total_sent += bytes_sent;
    }
    return false;
}
