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

#ifndef _ZSTACK_H
#define _ZSTACK_H

#include "zarray.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef zarray_t zstack_t;

#define zstack_create zarray_create
#define zstack_destroy zarray_destroy
#define zstack_vmap zarray_vmap
#define zstack_map zarray_map
#define zstack_push(zs, o) zarray_add(zs, o)

static void zstack_pop(zstack_t *zs, void *p)
{
    int idx = zarray_size(zs) - 1;
    zarray_get(zs, idx, p);
    zarray_remove_index(zs, idx, 0); // 0: no shuffle
}

#define zstack_empty(zs) (zarray_size(zs)==0)
#define zstack_size(zs) zarray_size(zs)

#ifdef __cplusplus
}
#endif

#endif

