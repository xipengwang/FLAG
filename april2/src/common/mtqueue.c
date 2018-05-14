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

#include <pthread.h>
#include <stdlib.h>

#include "mtqueue.h"

mtqueue_t *mtqueue_create()
{
    mtqueue_t *q = calloc(1, sizeof(mtqueue_t));

    pthread_mutexattr_t mattr;
    pthread_mutexattr_init(&mattr);
    pthread_mutexattr_settype(&mattr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&q->mutex, &mattr);
    pthread_cond_init(&q->cond, NULL);
    return q;
}

void mtqueue_destroy(mtqueue_t *q)
{
    pthread_mutex_destroy(&q->mutex);
    pthread_cond_destroy(&q->cond);

    free(q->els);
    free(q);
}

int mtqueue_size(mtqueue_t *q)
{
    int sz;

    pthread_mutex_lock(&q->mutex);
    sz = q->sz;
    pthread_mutex_unlock(&q->mutex);
    return sz;
}

void mtqueue_put(mtqueue_t *q, void *p)
{
    pthread_mutex_lock(&q->mutex);

    if (q->sz == q->alloc) {
        // realloc. We'll allocate a new
        int newalloc = 2 * q->alloc;
        if (newalloc < 16)
            newalloc = 16;

        void **newels = malloc(newalloc * sizeof(void*));

        // copy current elements into newels
        for (int i = 0; i < q->sz; i++)
            newels[i] = q->els[(q->get_pos + i) % q->alloc];

        free(q->els);
        q->els = newels;
        q->alloc = newalloc;
        q->put_pos = q->sz;
        q->get_pos = 0;
    }

    q->els[q->put_pos] = p;
    q->put_pos = (q->put_pos + 1) % q->alloc;
    q->sz++;

    pthread_cond_signal(&q->cond);
    pthread_mutex_unlock(&q->mutex);
}

void *mtqueue_get_block(mtqueue_t *q)
{
    void *p;

    pthread_mutex_lock(&q->mutex);

    while (q->sz == 0)
        pthread_cond_wait(&q->cond, &q->mutex);

    p = q->els[q->get_pos];
    q->get_pos = (q->get_pos + 1) % q->alloc;
    q->sz--;

    pthread_mutex_unlock(&q->mutex);

    return p;
}

void *mtqueue_get_nonblock(mtqueue_t *q)
{
    void *p = NULL;

    pthread_mutex_lock(&q->mutex);

    if (q->sz) {
        p = q->els[q->get_pos];
        q->get_pos = (q->get_pos + 1) % q->alloc;
        q->sz--;
    }

    pthread_mutex_unlock(&q->mutex);

    return p;
}

void mtqueue_lock(mtqueue_t *q)
{
    pthread_mutex_lock(&q->mutex);
}

void *mtqueue_locked_get(mtqueue_t *q, int idx)
{
    return q->els[(q->get_pos + idx) % q->alloc];
}

void mtqueue_locked_remove(mtqueue_t *q, int idx)
{
    int i = (q->get_pos + idx) % q->alloc;

    // copy the entries that follow.
    // (If the queue has 10 elements, and we remove the one at idx 0, we should
    // copy 9 elements.)
    for (int j = 0; j < q->sz - idx - 1; j++) {
        q->els[(i + j) % q->alloc] = q->els[(i + j + 1) % q->alloc];
    }

    q->sz--;
    q->put_pos = (q->put_pos + q->alloc - 1) % q->alloc;
}

void mtqueue_unlock(mtqueue_t *q)
{
    pthread_mutex_unlock(&q->mutex);
}
