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

#ifndef _ESTREAM_T
#define _ESTREAM_T

#include <stdint.h>

typedef struct estream estream_t;

struct estream
{
    // users should only read this; it will be written by the implementation.
    int closed;

    // returns negative if error, else returns the total length written
    int (*writev_fully)(estream_t *es, int vlen, const void **bufs, uint64_t *lens);

    // returns the number of bytes read, negative on error (timeout is
    // an error). Negative timeout means forever, zero means poll.
    int (*read_timeout)(estream_t *es, void *buf, int maxsize, int timeout_ms);

    // Calling closed sets closed = 1.
    int (*close)(estream_t *es);

    void (*destroy)(estream_t *es);

    void *user;
    int user_fd;
};

estream_t *estream_create_from_fd(int fd);

int estream_printf(estream_t *stream, const char *fmt, ...);

// returns zero on success, negative on error or timeout. The total
// number of bytes read (even if a timeout occured) is written to
// readlen, which can be NULL. A line is terminated by \n. \r will be
// stripped out.
int estream_read_line_timeout(estream_t *stream, void *_buf, int maxlen, int timeout_ms, int *readlen);

// read exactly 'len' bytes, returning negative if anything bad happens (error or timeout)
int estream_read_fully_timeout(estream_t *stream, void *_buf, int len, int timeout_ms);

#endif
