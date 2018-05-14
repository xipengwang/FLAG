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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <inttypes.h>
#include <poll.h>
#include <netdb.h>
#include <unistd.h>

#include "net_util.h"
#include "string_util.h"
#include "time_util.h"
#include "sha1.h"
#include "encode_bytes.h"

// Create a UDP socket for transmitting.
// @addr is the FROM address. Can be NULL for any arbitrary interface.
//
// returns < 0 on error.
int udp_socket_create(struct sockaddr_in *addr, int broadcast)
{
    // create UDP socket
    int fd = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd < 0) {
        perror("socket");
        return -1;
    }

    int v = 1;
    if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &v, sizeof(v))) {
        perror("setsockopt");
        return -1;
    }

    if (broadcast) {
        v = 1;
        if (setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &v, sizeof(v))) {
            perror("setsockopt");
            return -1;
        }
    }

    if (addr) {
        if (bind(fd, (struct sockaddr*) addr, sizeof(struct sockaddr_in))) {
            perror("bind");
            return -1;
        }
    }

    return fd;
}

/** make and bind a udp socket to a specified port. Returns the fd. **/
int udp_socket_listen(int port)
{
    int sock = socket(PF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
        return -1;

    int v = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &v, sizeof(v))) {
        perror("setsockopt");
        return -2;
    }

    struct sockaddr_in listen_addr;
    memset(&listen_addr, 0, sizeof(struct sockaddr_in));
    listen_addr.sin_family = AF_INET;
    listen_addr.sin_port = htons(port);
    listen_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    int res = bind(sock, (struct sockaddr*) &listen_addr, sizeof(listen_addr));
    if (res < 0)
        return -3;

    return sock;
}


/** return the local port number for a socket. **/
int udp_socket_get_port(int sock)
{
    struct sockaddr_in addr;
    socklen_t addr_len = sizeof(addr);

    int res = getsockname(sock, (struct sockaddr*) &addr, &addr_len);

    if (res < 0)
        return -1;

    return ntohs(addr.sin_port);
}

// return 0 on success
int udp_send(const char *ipaddr, int port, void *data, int datalen)
{
    // fill in address structure
    struct sockaddr_in remote_addr;

    memset(&remote_addr, 0, sizeof(struct sockaddr_in));
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_port = htons(port);

    struct hostent *host = gethostbyname(ipaddr);
    if (host == NULL) {
        printf("Couldn't resolve host %s\n", ipaddr);
        return -1;
    }

    memcpy(&remote_addr.sin_addr.s_addr, host->h_addr_list[0], host->h_length);

    // now actually send.
    // TODO: Could try to guess if ipaddr is a broadcast
    int sock = udp_socket_create(NULL, 0);
    ssize_t res = sendto(sock, data, datalen, 0, (struct sockaddr*) &remote_addr, sizeof(remote_addr));

    // cleanup
    close(sock);
    return res != datalen;
}

int ip4_parse_addr(const char *s, struct sockaddr_in *addr, int default_port, int *_actual_port)
{
    zarray_t *toks = str_split(s, ":");

    memset(addr, 0, sizeof(struct sockaddr_in));

    int actual_port = default_port;
    if (zarray_size(toks) == 2) {
        const char *tok1;
        zarray_get(toks, 1, &tok1);
        actual_port = atoi(tok1);
    }

    addr->sin_port = htons(actual_port);
    addr->sin_family = AF_INET;

    if (_actual_port)
        *_actual_port = actual_port;

    const char *tok0;
    zarray_get(toks, 0, &tok0);

    if (inet_pton(AF_INET, tok0, &addr->sin_addr.s_addr) != 1)
        return -1;

    zarray_vmap(toks, free);
    zarray_destroy(toks);

    return 0;
}
