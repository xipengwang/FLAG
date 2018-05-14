/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.

This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, see <http://www.gnu.org/licenses/>.
*/

#include <assert.h>
#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <fcntl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <lcm/lcm.h>

#include "common/getopt.h"
#include "common/io_util.h"
#include "common/time_util.h"

static int running = 1;

typedef struct state state_t;
struct state {
    lcm_t *lcm;

    double rate;
    int client_sock;
    int sz;
    char *buf;

    pthread_t lcm_thread;
    pthread_t tx_thread;
    pthread_mutex_t lock;
};

void signal_cb(int signal)
{
    switch (signal) {
        case SIGINT:
            printf("SIGINT: Stop running\n");
            running = 0;
            break;
        default:
            printf("Signal %d not handled\n", signal);
    }
}

void *lcm_loop(void *user)
{
    state_t *state = user;

    while (running) {
        lcm_handle_timeout(state->lcm, 1000);
    }

    return NULL;
}

void lcm_cb(const lcm_recv_buf_t *rbuf,
              const char *channel,
              void *user)
{
    state_t *state = user;

    pthread_mutex_lock(&state->lock);

    if (state->buf)
        free(state->buf);
    state->sz = rbuf->data_size;
    state->buf = malloc(state->sz);
    memcpy(state->buf, rbuf->data, state->sz);

    pthread_mutex_unlock(&state->lock);
}

void *tx_loop(void *user)
{
    state_t *state = user;

    char *buf = NULL;
    while (running) {
        timeutil_usleep((int)(1000000/state->rate));

        pthread_mutex_lock(&state->lock);
        buf = NULL;
        if (state->buf) {
            buf = state->buf;
            state->buf = NULL;
        }
        pthread_mutex_unlock(&state->lock);

        if (buf == NULL)
            continue;
        assert (buf);

        int hsz = htonl(state->sz);
        int err = 0;
        if (write_fully(state->client_sock, &hsz, sizeof(hsz)) < 0) {
            printf("ERR: Writing sz to socket\n");
            err = 1;
        }

        if (write_fully(state->client_sock, buf, state->sz) < 0) {
            printf("ERR: Writing buf to socket\n");
            err = 1;
        }

        free(buf);

        if (err)
            break;
    }

    close(state->client_sock);
    return NULL;
}

int main(int argc, char **argv)
{
    setlinebuf(stderr);
    setlinebuf(stdout);
    signal(SIGINT, signal_cb);
    signal(SIGPIPE, SIG_IGN);

    getopt_t *gopt = getopt_create();

    getopt_add_bool(gopt, 'h', "help", 0, "Show usage");
    getopt_add_int(gopt, 'p', "port", "8989", "Port #");
    getopt_add_double(gopt, '\0', "rate", "1", "TX rate/sec");
    getopt_add_string(gopt, '\0', "channel", "IMAGE", "LCM Channel");

    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt, "help")) {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(gopt);
        exit(1);
    }

    state_t *state = calloc(1, sizeof(state_t));
    state->lcm = lcm_create(NULL);
    state->rate = getopt_get_double(gopt, "rate");
    state->buf = NULL;
    state->sz = 0;

    lcm_subscribe(state->lcm, getopt_get_string(gopt, "channel"), lcm_cb, state);

    pthread_mutex_init(&state->lock, NULL);
    pthread_create(&state->lcm_thread, NULL, lcm_loop, state);

    // Set up to listen for TCP image client
    int sockfd, clientfd;
    struct sockaddr_in server_addr = {0};
    struct sockaddr_in client_addr = {0};

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        perror("ERR: sockopen");
        return -1;
    }
    int option = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (char*)&option, sizeof(option));

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(getopt_get_int(gopt, "port"));
    if (bind(sockfd, (struct sockaddr *) &server_addr, sizeof(server_addr)) < 0) {
        perror("ERR: bind");
        return -1;
    }

    listen(sockfd, 1);  // Only accept 1 connection at a time
    fcntl(sockfd, F_SETFL, O_NONBLOCK);

    socklen_t client_len = sizeof(client_addr);
    while (running) {
        clientfd = accept(sockfd, (struct sockaddr *) &client_addr, &client_len);
        if (clientfd < 0 && (errno == EWOULDBLOCK || errno == EAGAIN)) {
            sleep(1);
            continue;
        }

        printf("Connection accepted at %d\n", clientfd);
        // We only ever allow one thread at a time
        pthread_join(state->tx_thread, NULL);

        state->client_sock = clientfd;
        pthread_create(&state->tx_thread, NULL, tx_loop, state);
    }


    pthread_join(state->tx_thread, NULL);
    close(sockfd);
    lcm_destroy(state->lcm);
    getopt_destroy(gopt);
    if (state->buf)
        free(state->buf);
    free(state);

    return 0;
}
