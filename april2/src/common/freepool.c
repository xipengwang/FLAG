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
#include <string.h>

#include "freepool.h"
#include "zarray.h"

struct freepool_entry
{
    void *p;
    void (*destroy)(void *p);
};

struct freepool
{
    zarray_t *za; // struct freepool_entry
};

freepool_t *freepool_create()
{
    freepool_t *fp = calloc(1, sizeof(freepool_t));
    fp->za = zarray_create(sizeof(struct freepool_entry));

    return fp;
}

void *freepool_add(freepool_t *fp, void *p, void (*destroy)())
{
    struct freepool_entry e;
    e.p       = p;
    e.destroy = destroy;

    zarray_add(fp->za, &e);

    return p;
}

void freepool_destroy(freepool_t *fp)
{
    struct freepool_entry e;

    for (int i = 0; i < zarray_size(fp->za); i++) {
        zarray_get(fp->za, i, &e);

        // checking for NULL allows user to wrap function calls with
        // macros that add the pointer to freepool before any value checking.
        if(e.p != NULL)
            e.destroy(e.p);
    }

    zarray_destroy(fp->za);
    memset(fp, 0, sizeof(freepool_t));
    free(fp);
}
