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
#include <stddef.h>
#include <string.h>
#include <stdint.h>
#include <assert.h>
#include "common/doubles.h"
#include "common/zarray.h"
#include "interpolator.h"

interpolator_t *interpolator_create(int struct_size, int utime_offset, double history_time, int64_t max_utime_error)
{
    interpolator_t *interp = calloc(1, sizeof(interpolator_t));
    interp->struct_size = struct_size;
    interp->utime_offset = utime_offset;
    interp->fields = NULL;
    interp->history_time = history_time;
    interp->max_utime_error = max_utime_error;

    interp->data = zarray_create(sizeof(void*));
    return interp;
}

void interpolator_destroy(interpolator_t *interp)
{
    zarray_destroy(interp->data);
    free(interp);
}

void interpolator_add_field(interpolator_t *interp, int type, int length, int offset)
{
    interp->fields = realloc(interp->fields, sizeof(struct interpolator_field) * (interp->nfields + 1));
    interp->fields[interp->nfields].type = type;
    interp->fields[interp->nfields].length = length;
    interp->fields[interp->nfields].offset = offset;
    interp->nfields++;
}

void interpolator_add(interpolator_t *interp, const void *_obj)
{
    char *copy = malloc(interp->struct_size);
    memcpy(copy, _obj, interp->struct_size);

    zarray_add(interp->data, &copy);

    int64_t obj_utime = *((int64_t*) &copy[interp->utime_offset]);

    if(obj_utime > interp->largest_utime)
        interp->largest_utime = obj_utime;

    while (1) {
        char *old = NULL;
        zarray_get(interp->data, 0, &old);
        int64_t old_utime = *((int64_t*) &old[interp->utime_offset]);

        if (obj_utime - old_utime < interp->history_time * 1E6)
            break;

        free(old);
        zarray_remove_index(interp->data, 0, 0);
    }
}

// return goodness?
int interpolator_get(interpolator_t *interp, int64_t utime, void *_out)
{
    char *out = _out;
    memset(_out, 0, interp->struct_size);

    char *a = NULL; // the latest item before utime
    int64_t a_utime = 0;
    char *b = NULL; // the earliest item after utime
    int64_t b_utime = 0;

    for (int i = 0; i < zarray_size(interp->data); i++) {
        char *d;
        zarray_get(interp->data, i, &d);
        int64_t d_utime;
        d_utime = *((uint64_t*) &d[interp->utime_offset]);

        if (d_utime == utime) {
            memcpy(out, d, interp->struct_size);
            return 0;
        }

        if (d_utime < utime) {
            if (a_utime == 0 || (utime - d_utime) < (utime - a_utime)) {
                a_utime = d_utime;
                a = d;
            }
        }

        if (d_utime > utime) {
            if (b_utime == 0 || (d_utime - utime) < (b_utime - utime)) {
                b_utime = d_utime;
                b = d;
            }
        }
    }

    if (utime - a_utime > interp->max_utime_error)
        a_utime = 0;

    if (b_utime - utime > interp->max_utime_error)
        b_utime = 0;

    if (a_utime == 0 && b_utime == 0)
        return -2;

    if (a_utime == 0) {
        memcpy(out, b, interp->struct_size);
        return -1;
    }

    if (b_utime == 0) {
        memcpy(out, a, interp->struct_size);
        return -1;
    }

    assert(a_utime != 0 && b_utime != 0);

    double a_alpha = 1.0 * (b_utime - utime) / (b_utime - a_utime);
    double b_alpha = 1.0 * (utime - a_utime) / (b_utime - a_utime);

    *((uint64_t*) &out[interp->utime_offset]) = utime;
    for (int i = 0; i < interp->nfields; i++) {
        switch (interp->fields[i].type) {
            case INTERPOLATOR_DOUBLE_LINEAR: {
                double *avs = (double*) &a[interp->fields[i].offset];
                double *bvs = (double*) &b[interp->fields[i].offset];
                double *outvs = (double*) &out[interp->fields[i].offset];

                for (int j = 0; j < interp->fields[i].length; j++)
                    outvs[j] = avs[j]*a_alpha + bvs[j]*b_alpha;
                break;
            }

            case INTERPOLATOR_DOUBLE_QUAT: {
                assert(interp->fields[i].length == 4);

                double *avs = (double*) &a[interp->fields[i].offset];
                double *bvs = (double*) &b[interp->fields[i].offset];
                double *outvs = (double*) &out[interp->fields[i].offset];

                doubles_quat_slerp(avs, bvs, outvs, b_alpha);
                break;
            }

            case INTERPOLATOR_DOUBLE_RADIANS: {
                double *avs = (double*) &a[interp->fields[i].offset];
                double *bvs = (double*) &b[interp->fields[i].offset];
                double *outvs = (double*) &out[interp->fields[i].offset];

                for (int j = 0; j < interp->fields[i].length; j++) {
                    double av = avs[j];
                    double bv = mod2pi_ref(av, bvs[j]);

                    outvs[j] = av*a_alpha + bv*b_alpha;
                }
                break;
            }

            default:
                assert(0);
        }
    }

    return 0;
}
