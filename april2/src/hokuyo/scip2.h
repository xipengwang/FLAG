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

#ifndef _SCIP2_H
#define _SCIP2_H

#include <stdio.h>
#include <pthread.h>
#include <stdint.h>
#include "common/zhash.h"
#include "common/zarray.h"

#define RXBUF_SIZE 4096

typedef struct scip2 scip2_t;
struct scip2
{
    int fd;

    int debug;

    pthread_t reader_thread;
    uint32_t xid;

    zhash_t *transactions; // (void*) xid-> scip2_transaction_t

    pthread_mutex_t mutex; // protects writes on fd and xid, and transactions hash table.

    void (*on_99_data)(zarray_t *response, void *user);
    void *on_99_data_user;

    char rxbuf[RXBUF_SIZE];
    int rxbuf_pos, rxbuf_avail;

};

scip2_t *scip2_create(const char *path);

zarray_t *scip2_transaction(scip2_t *scip, const char *command, int timeoutms);
void scip2_set_99_data_handler(scip2_t *scip, void (*on_99_data)(zarray_t *response, void *user), void *user);

void scip2_response_free(scip2_t *scip, zarray_t *response);

#endif
