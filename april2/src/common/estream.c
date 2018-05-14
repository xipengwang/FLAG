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
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <poll.h>
#include "estream.h"

int estream_printf(estream_t *stream, const char *fmt, ...)
{
    va_list ap;
    int tmpalloc = 256;
    char *tmp = malloc(tmpalloc);

    va_start(ap, fmt);
    int n = vsnprintf(tmp, tmpalloc, fmt, ap);
    va_end(ap);

    if (n >= tmpalloc) {
        tmp = realloc(tmp, n + 1);
        va_start(ap, fmt);
        vsprintf(tmp, fmt, ap);
        va_end(ap);
    }

    int ret = stream->writev_fully(stream, 1, (const void*[]) { tmp }, (uint64_t[]) { n });
    free(tmp);
    return ret;
}

int estream_read_line_timeout(estream_t *stream, void *_buf, int maxlen, int timeout_ms, int *readlen)
{
    int len = 0;
    char *buf = (char*) _buf;
    int ret = -1;

    while (1) {

      if (len + 1 == maxlen) { // buffer full?
          ret = -2;
          goto cleanup;
      }

        int thislen = stream->read_timeout(stream, &buf[len], 1, timeout_ms);

        if (thislen < 0) { // read error?
            ret = -3;
            goto cleanup;
        }

        if (thislen == 0) { // timeout
            ret = -4;
            goto cleanup;
        }

        if (buf[len]=='\n') { // end of line?
            ret = 0;
            goto cleanup;
        }

        // silently swallow \r.
        if (buf[len] != '\r')
            len++;
    }

  cleanup:
    buf[len] = 0;

    *readlen = len;
    return ret;
}

static int fd_writev_fully(estream_t *es, int vlen, const void **bufs, uint64_t *lens)
{
    int total_written = 0;

    for (int i = 0; i < vlen; i++) {
        int written = 0;
        const char *buf = bufs[i];

        while (written < lens[i]) {
            int this_written = write(es->user_fd, &buf[written], lens[i] - written);
            if (this_written <= 0)
                return -1;
            written += this_written;
        }

        total_written += written;
    }

    return total_written;
}

// returns the number of bytes read, negative on error.
static int fd_read_timeout(estream_t *es, void *_buf, int maxsize, int timeout_ms)
{
    char *buf = _buf;
    struct pollfd pfd;

    pfd.fd = es->user_fd;
    pfd.events = POLLIN;

    int res = poll(&pfd, 1, timeout_ms);
    if (res < 0) // error
        return -1;

    if (res == 0) { // timeout
        return -1;
    }

    ssize_t readlen = read(es->user_fd, buf, maxsize);
    if (readlen < 0) {
        return -1;
    }

    if (readlen == 0) {
        return -1;
    }

    return readlen;
}

// Calling closed sets closed = 1.
static int fd_close(estream_t *es)
{
    if (es->closed)
        return -1;

    es->closed = 1;
    close(es->user_fd);

    return 0;
}

static void fd_destroy(estream_t *es)
{
    free(es);
}

estream_t *estream_create_from_fd(int fd)
{
    estream_t *estream = calloc(1, sizeof(estream_t));
    estream->writev_fully = fd_writev_fully;
    estream->read_timeout = fd_read_timeout;
    estream->close = fd_close;
    estream->destroy = fd_destroy;
    estream->user_fd = fd;

    return estream;
}

int estream_read_fully_timeout(estream_t *es, void *_buf, int len, int timeout_ms)
{
    char *buf = _buf;
    int total_read = 0;

    while (total_read < len) {
        int this_read = es->read_timeout(es, &buf[total_read], len - total_read, timeout_ms);
        if (this_read <= 0) {
//            printf("read before error; %d\n", total_read);
            return -1;
        }
        total_read += this_read;
    }

    return total_read;
}
