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
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include "mati.h"

#define TYPE int32_t

mati_t * mati_create(int nrows, int ncols)
{
    assert(nrows >= 0);
    assert(ncols >= 0);

    mati_t *m = malloc(sizeof(mati_t));
    if (m == NULL) {
        fprintf(stderr, "Malloc for mati_t return NULL\n");
        exit(EXIT_FAILURE);
    }

    m->data = malloc(nrows * ncols * sizeof(int));
    if (m->data == NULL) {
        fprintf(stderr, "Malloc for mati_t return NULL\n");
        exit(EXIT_FAILURE);
    }

    m->nrows = nrows;
    m->ncols = ncols;

    memset(m->data, 0, nrows * ncols * sizeof(int));

    return m;
}

void mati_destroy(mati_t *m)
{
    if (m->data != NULL)
        free(m->data);

    // set to NULL to cause segfault (more often) if mati is used
    // after the destroy call (hard to catch failure mode)
    m->data = NULL;

    memset(m, 0, sizeof(mati_t));
    free(m);
}

// row and col are zero-based
int mati_get(const mati_t *m, int row, int col)
{
    // data in row-major order (index = rows*ncols + col)
    assert(m != NULL);
    assert(row >= 0);
    assert(row < m->nrows);
    assert(col >= 0);
    assert(col < m->ncols);

    return (m->data[row * m->ncols + col]);
}

mati_t * mati_copy(const mati_t *m)
{
    assert(m != NULL);

    mati_t *x = mati_create(m->nrows, m->ncols);
    memcpy(x->data, m->data, sizeof(TYPE)*m->ncols*m->nrows);

    return x;

}


// row and col are zero-based
void mati_put(mati_t *m, int row, int col, int value)
{
    // data in row-major order (index = rows*ncols + col)
    assert(m != NULL);
    assert(row >= 0);
    assert(row < m->nrows);
    assert(col >= 0);
    assert(col < m->ncols);

    m->data[row * m->ncols + col] = value;
}
