#include <unistd.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <string.h>
#include <stdio.h>

#include "Network.hh"

#include "defines.hh"

bool Network::socket_init = false;
bool Network::thread_running = false;
bool Network::thread_should_run = false;
int Network::n_fd = 0;
pthread_t Network::thread;

void Network::init()
{
    socket_init = false;
    thread_running = false;
    thread_should_run = false;
    engine = eng;
    n_fd = 0;
    memset(&thread, 0, sizeof(pthread_t));

    if((n_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
            printf("Error creating socket\n");
            return;
        }

        struct sockaddr_in n_addr;

        memset((char*)&n_addr, 0, sizeof(n_addr));

        n_addr.sin_family = AF_INET;
        n_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        n_addr.sin_port = htons(SERVER_PORT);

        if(bind(n_fd, (struct sockaddr*)&n_addr, sizeof(n_addr)) < 0) {
                printf("Socket binding failed\n");
                return;
        }

    socket_init = true;
}

Network::Network()
{

}

bool Network::sendPacket(ip_t ip, unsigned char *buf, size_t buf_len)
{
    if(!socket_init)
        return false;

    struct sockaddr_in dest_addr;
        memset((char*)&dest_addr, 0, sizeof(dest_addr));
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(CLIENT_PORT);
        dest_addr.sin_addr.s_addr = ip;

        return (sendto(n_fd, buf, buf_len, 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr)) > -1);
}
