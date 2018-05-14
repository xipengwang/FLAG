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

#include <stdlib.h>

#include "expiry_cache.h"
#include "common/time_util.h"

struct expiry_cache
{
    int64_t rcv_utime;
    int64_t msg_utime;
    void *  msg;
    void *  (*copy)(const void * orig);
    void    (*destructor)(void * destuctee);
};


expiry_cache_t * expiry_cache_create(void * (*copy)(const void * orig), void (*destructor)(void * destuctee))
{
    expiry_cache_t * ec = calloc(1, sizeof(expiry_cache_t));
    ec->copy = copy;
    ec->destructor = destructor;
    return ec;
}

void expiry_cache_destroy(expiry_cache_t * ec)
{
    if(ec->msg != NULL)
        ec->destructor(ec->msg);
    free(ec);
}

void expiry_cache_add(expiry_cache_t * ec, const void * new_msg)
{
    if(ec->msg != NULL)
        ec->destructor(ec->msg);

    ec->msg         = ec->copy(new_msg);
    ec->rcv_utime   = utime_now();
}

void * expiry_cache_get(expiry_cache_t * ec, int64_t time_diff_max)
{
    if(ec == NULL) return NULL;

    if (time_diff_max < 0)
        return ec->msg;

    int64_t dt  = utime_now() - ec->rcv_utime;
    if(dt > 0 && dt < time_diff_max)
        return ec->msg;

    if(ec->msg != NULL)
    {
        ec->destructor(ec->msg);
        ec->msg = NULL;
    }

    return NULL;
}

void expiry_cache_add_mono(expiry_cache_t * ec, const void * new_msg, int64_t utime)
{
    if(utime > ec->msg_utime)
    {
        ec->msg_utime = utime;
        expiry_cache_add(ec, new_msg);
    }
}

int64_t expiry_cache_get_msg_utime(expiry_cache_t * ec)
{
    if (ec == NULL) return 0;
    return ec->msg_utime;
}

int64_t expiry_cache_get_age(expiry_cache_t * ec)
{
    if (ec == NULL) return 0;
    return utime_now() - ec->rcv_utime;
}
