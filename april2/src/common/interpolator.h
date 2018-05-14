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

#ifndef _INTERPOLATOR_H
#define _INTERPOLATOR_H

#include <stdint.h>

#include "common/zarray.h"

enum { INTERPOLATOR_DOUBLE_LINEAR, INTERPOLATOR_DOUBLE_QUAT, INTERPOLATOR_DOUBLE_RADIANS };

struct interpolator_field
{
    int type, length;
    int offset;
};

typedef struct interpolator interpolator_t;
struct interpolator
{
    int64_t max_utime_error;

    int struct_size;
    int utime_offset;
    int nfields;
    double history_time;
    struct interpolator_field *fields;

    zarray_t *data;

    int64_t largest_utime;
};


interpolator_t *interpolator_create(int struct_size, int utime_offset, double history_time, int64_t max_utime_error);

void interpolator_destroy(interpolator_t *interp);

void interpolator_add_field(interpolator_t *interp, int type, int length, int offset);

void interpolator_add(interpolator_t *interp, const void *obj);

// returns 0 if a double-sided interpolation was possible. Returns -1
// if only one-sided, -2 on failure.
int interpolator_get(interpolator_t *interp, int64_t utime, void *_out);
#endif
