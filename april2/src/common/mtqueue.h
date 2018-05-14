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

#ifndef _MTQUEUE_H
#define _MTQUEUE_H

#include <pthread.h>

typedef struct mtqueue mtqueue_t;

struct mtqueue
{
    int sz;
    int alloc;

    int get_pos; // circular buffer
    int put_pos;
    void **els;

    pthread_mutex_t mutex;
    pthread_cond_t cond;
};

mtqueue_t *mtqueue_create();
void mtqueue_destroy(mtqueue_t *q);

int mtqueue_size(mtqueue_t *q);

void mtqueue_put(mtqueue_t *q, void *p);

void *mtqueue_get_block(mtqueue_t *q);

void *mtqueue_get_nonblock(mtqueue_t *q);

void mtqueue_lock(mtqueue_t *q);
void *mtqueue_locked_get(mtqueue_t *q, int idx);
void mtqueue_locked_remove(mtqueue_t *q, int idx);
void mtqueue_unlock(mtqueue_t *q);

#endif
