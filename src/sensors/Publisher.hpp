#include "socket.hpp"
#include <condition_variable>
#include <memory>
#include <mutex>
#include <netinet/in.h>
#include <queue>
#include <shared_mutex>
#include <vector>

struct KinematicsData {
    double position[3];     // x, y, z
    double velocity[3];     // vx, vy, vz
    double acceleration[3]; // ax, ay, az
    std::chrono::steady_clock::time_point timestamp;
    uint64_t sequence_id;
};

class Publisher
{
    // std::vector<int> socket_fds;
    std::vector<TCP_Socket> sockets;
    std::queue<PVAT> send_queue;
    std::thread sender_thread;

    void publish();
};

typedef struct {
    PVAT data;
    int socket_count;
    int sockets[MAX_SOCKETS];
    // Optional: timestamp, priority, etc.
} queued_message_t;
