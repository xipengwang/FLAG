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

#ifndef _EXPIRY_CACHE_H
#define _EXPIRY_CACHE_H

#include <inttypes.h>

/**
 * A simple single-message cache
 * by default, the cache makes a copy of what you pass in,
 * but returns a 'live' pointer
 *
 * Construction is a bit gnarly:
 * img_cache = expiry_cache_create((void *(*)(const void*)) image_t_copy, (void (*)(void*)) image_t_destroy);
 */

typedef struct expiry_cache expiry_cache_t;

expiry_cache_t * expiry_cache_create(void * (*copy)(const void * orig), void (*destructor)(void * destuctee));

void expiry_cache_destroy(expiry_cache_t * ec);

//The cace will
void expiry_cache_add(expiry_cache_t * ec, const void * msg);

//enforces monotonicity using the utime you pass in
void expiry_cache_add_mono(expiry_cache_t * ec, const void * new_msg, int64_t utime);

//Still owned by the cache!
void * expiry_cache_get(expiry_cache_t * ec, int64_t time_diff_max);

//User Owned
void * expiry_cache_get_owned(expiry_cache_t * ec, int64_t time_diff_max);

int64_t expiry_cache_get_msg_utime(expiry_cache_t * ec);

int64_t expiry_cache_get_age(expiry_cache_t * ec);

#endif
