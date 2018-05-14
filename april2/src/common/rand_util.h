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

#ifndef _RAND_UTIL_H
#define _RAND_UTIL_H

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

static inline float randf_uniform(double min, double max)
{
    double t = ((float) rand()) / RAND_MAX;

    return min + (max-min)*t;
}

// sample uniformly from [min, toobig)
static inline int randi_uniform(int min, int toobig)
{
    double t = rand() - 1;

    return min + (toobig - min)*t / RAND_MAX;
}

static inline float randf_normal(void)
{
    float s = 0.0f;
    float u, v, R2;

    while (s == 0.0f || s >= 1.0f)
    {
        // Sample two values uniformly from (-1, +1)
        u = ((float) rand()) * 2 / RAND_MAX - 1.0f;
        v = ((float) rand()) * 2 / RAND_MAX - 1.0f;

        R2 = u*u + v*v;
        s = R2;
    }

    float factor = sqrt( (-2*log(s)) / (s) );

    // both are normally distributed
    float z0 = u * factor;
    //float z1 = v * factor;

    // we only want to return one, for convenience
    return z0;
}

#ifdef __cplusplus
}
#endif

#endif
