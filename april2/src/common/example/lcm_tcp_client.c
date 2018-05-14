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
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>

#include <arpa/inet.h>
#include <lcm/lcm.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>

#include "common/getopt.h"
#include "common/io_util.h"

static int running = 1;

void signal_cb(int signal)
{
    switch (signal) {
        case SIGINT:
            printf("SIGINT: stop running\n");
            running = 0;
            break;
        default:
            break;
    }
}

int main(int argc, char **argv)
{
    setlinebuf(stderr);
    setlinebuf(stdout);
    signal(SIGINT, signal_cb);

    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Show usage");
    getopt_add_int(gopt, 'p', "port", "8989", "Server port #");
    getopt_add_string(gopt, 's', "server", "127.0.0.1", "Server IP");
    getopt_add_string(gopt, '\0', "channel", "IMAGE", "LCM Image channel");

    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt, "help")) {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(gopt);
        exit(1);
    }

    int sockfd;
    struct sockaddr_in server_addr = {0};

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        perror("ERR: socket");
        return -1;
    }

    server_addr.sin_family = AF_INET;
    inet_pton(AF_INET, getopt_get_string(gopt, "server"), &server_addr.sin_addr.s_addr);
    server_addr.sin_port = htons(getopt_get_int(gopt, "port"));

    printf("Connecting to server...\n");
    if (connect(sockfd, (struct sockaddr *) &server_addr, sizeof(server_addr)) < 0) {
        perror("ERR: connect");
        return -1;
    }

    lcm_t *lcm = lcm_create(NULL);

    while (running) {
        // Read size
        int sz;
        if (read_fully(sockfd, (char *)&sz, sizeof(sz)) < 0) {
            perror("ERR: sz read");
            return -1;
        }

        sz = ntohl(sz);
        printf("Got size: %d\n", sz);

        char buf[sz];
        if (read_fully(sockfd, buf, sizeof(buf)) < 0) {
            perror("ERR: buf read");
            return -1;
        }

        lcm_publish(lcm, getopt_get_string(gopt, "channel"), buf, sz);
    }

    printf("Shutting down...\n");
    close(sockfd);
    lcm_destroy(lcm);
    getopt_destroy(gopt);

    return 0;
}
