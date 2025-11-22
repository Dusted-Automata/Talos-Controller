
#include "brain.hpp"
#include "robot.hpp"
#include <iostream>


enum Command {
    CMD_START,
    CMD_STOP,
    CMD_PAUSE,
    CMD_CONTINUE,
    CMD_GETPATH,
    CMD_UNKNOWN,
};

typedef struct {
    const char *key;
    size_t len;
    Command cmd;
} CommandEntry;

Command parse_command(const char *buf, size_t len) {
    static const CommandEntry table[] = {
        { "start",  5, CMD_START  },
        { "stop", 4, CMD_STOP },
        { "pause", 5, CMD_PAUSE },
        { "continue",  8, CMD_CONTINUE  },
        { "path",  4, CMD_GETPATH  }
    };

    const size_t tableSize = sizeof(table) / sizeof(table[0]);

    for (size_t i = 0; i < tableSize; i++) {
        if (table[i].len != len)
            continue;                           

        if (memcmp(buf, table[i].key, len) == 0) 
            return table[i].cmd;
    }

    return CMD_UNKNOWN;
}

void 
parse_cmds(Brain &brain) {


    struct sockaddr_in cliaddr;
    char buffer[1024];
    socklen_t len = sizeof(cliaddr);

    while (brain.running) {
        int n = recvfrom(brain.fd, &buffer, sizeof(buffer), 0, (struct sockaddr *)&cliaddr, &len);
        if (n < 0) {
            perror("recvfrom");
            continue;
        }

        Command cmd = parse_command(buffer, n);

        switch (cmd) {
            case CMD_START:
                break;
            case CMD_STOP:
                brain.robot->stop();
                break;
            case CMD_PAUSE:
                brain.robot->pause();
                std::cout << "robot.pause : " << brain.robot->paused << std::endl;
                break;
            case CMD_CONTINUE:
                brain.robot->resume();
                break;
            case CMD_GETPATH:
            case CMD_UNKNOWN: break;
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

bool
server_init(Brain &brain, Robot &r, int port)
{

    std::cout << "Starting Reader" << std::endl;
    if (brain.running) return true;
    brain.robot = &r;

    brain.fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (brain.fd < 0) { // TODO: propper logging
        perror("socket");
        return false;
    }

    memset(&brain.servaddr, 0, sizeof(brain.servaddr));
    brain.servaddr.sin_family = AF_INET;
    brain.servaddr.sin_addr.s_addr = INADDR_ANY;
    brain.servaddr.sin_port = htons(port);

    if (bind(brain.fd, (struct sockaddr*)&brain.servaddr, sizeof(brain.servaddr)) < 0) {
        perror("bind"); // TODO: propper logging
        return false;
    }

    brain.socket_thread = std::thread(&parse_cmds, std::ref(brain));
    brain.running = true;
    return true;
}

bool
server_deinit(Brain &brain){
    brain.running = false;
    brain.socket_thread.join();
    close(brain.fd);
    return true;
}

