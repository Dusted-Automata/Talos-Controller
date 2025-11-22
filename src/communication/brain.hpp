#pragma once
#include <netinet/in.h>
#include <thread>
#include <vector>

class Robot;

class Brain
{
  public:
    std::thread socket_thread;
    Robot *robot;
    std::vector<int> subscribers;
    std::atomic_bool running = false;
    struct sockaddr_in servaddr;
    int fd = -1;

    void loop();
};

void parse_cmds(Brain &server); 
bool server_init(Brain &server, Robot &r, int port);
bool server_deinit(Brain &server);
