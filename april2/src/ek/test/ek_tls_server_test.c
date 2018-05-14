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
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdint.h>
#include <netinet/in.h>
#include <poll.h>
#include <ctype.h>
#include <time.h>
#include <assert.h>
#include <pthread.h>

#include "../ek_tls.h"
#include "common/tcp_util.h"
#include "common/io_util.h"
#include "../ek_asn.h"

#define TIMEOUT_MS 10000

typedef struct state state_t;
struct state
{
    int listen_port;
    int listen_fd;

    ek_tls_server_t *server;
    pthread_t accept_thread;
};

struct reader_thread_info
{
    int fd;
    state_t *state;
};

void *reader_thread(void *_user)
{
    struct reader_thread_info *info = _user;
    state_t *state = info->state;

    estream_t *tcp_stream = estream_create_from_fd(info->fd);

    estream_t *tls_stream = ek_tls_server_conn_create(state->server, tcp_stream);

    while (1) {
        uint8_t buf[1024];

        int buflen = tls_stream->read_timeout(tls_stream, buf, sizeof(buf), -1);
        if (buflen < 0)
            break;

        for (int i = 0; i < buflen; i++)
            buf[i] = toupper(buf[i]);

        tls_stream->writev_fully(tls_stream, 1, (const void*[]) { (void*) buf }, (uint64_t[]) { buflen });
    }

    tls_stream->close(tls_stream);
    tls_stream->destroy(tls_stream);

    return NULL;
}

void *accept_thread(void *_user)
{
    state_t *state = _user;

    while (1) {
        struct sockaddr addr;
        socklen_t addr_len = sizeof(struct sockaddr);

        int fd = accept(state->listen_fd, &addr, &addr_len);
        if (fd < 0) {
            perror("accept");
            continue;
        }

        struct reader_thread_info *info = calloc(1, sizeof(struct reader_thread_info));
        info->fd = fd;
        info->state = state;

        pthread_t _reader_thread;
        pthread_create(&_reader_thread, NULL, reader_thread, info);
        pthread_detach(_reader_thread);

        // XXX limit # of threads for DoS
    }

    return NULL;
}

int main(int argc, char *argv[])
{
    signal(SIGPIPE, SIG_IGN);

    state_t *state = calloc(1, sizeof(state_t));

    state->server = ek_tls_server_create("privkey.der", "cacert.der");
    state->listen_port = 8124;

    state->listen_fd = tcp_util_listen(state->listen_port, 10, 0);
    if (state->listen_fd < 0) {
        free(state);
        printf("Unable to create socket\n");
        return 0;
    }

    pthread_create(&state->accept_thread, NULL, accept_thread, state);
    pthread_detach(state->accept_thread);

    while (1) {
        sleep(1);
    }
}
