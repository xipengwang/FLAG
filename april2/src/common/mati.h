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

#ifndef _MATS_H
#define _MATS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _mati_t mati_t;
struct _mati_t
{
    int nrows;
    int ncols;

    // data in row-major order (index = rows*ncols + col)
    int32_t *data;
};

mati_t * mati_create(int nrows, int ncols);
int mati_get(const mati_t *m, int row, int col);
mati_t * mati_copy(const mati_t *m);
void mati_put(mati_t *m, int row, int col, int value);
void mati_destroy(mati_t *m);

#ifdef __cplusplus
}
#endif

#endif
