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

#ifndef TIC_H
#define TIC_H

#include <stdint.h>
#include "common/time_util.h"

// Simple utility for timing code.

typedef struct
{
    int64_t init_utime;
    int64_t start_utime;
} tic_t;


// returns a tic object (by value) that immediately begins counting time
inline static tic_t tic_begin()
{
    tic_t tic;

    tic.init_utime = tic.start_utime = utime_now(); // blacklist-ignore

    return tic;
}

// get the current time (microseconds), and reset the time
inline static int64_t toctic_us(tic_t * tic)
{
    int64_t end_utime = utime_now(); // blacklist-ignore
    int64_t elapsed = end_utime - tic->start_utime;

    tic->start_utime = end_utime;

    return elapsed;
}

// get the current time (microseconds), but the time keeps counting
inline static int64_t toc_us(tic_t * tic)
{
    int64_t end_utime = utime_now(); // blacklist-ignore
    int64_t elapsed = end_utime - tic->start_utime;

    return elapsed;
}


// get the total time (microseconds) since tic_being();
inline static int64_t tic_total_us(tic_t * tic)
{
    int64_t end_utime = utime_now(); // blacklist-ignore
    int64_t elapsed = end_utime - tic->init_utime;

    return elapsed;
}

// get the current time (seconds), and reset the time
inline static double toctic_s(tic_t * tic)
{
    return toctic_us(tic) /1e6;
}

// get the current time (seconds), but the time keeps counting
inline static double toc_s(tic_t * tic)
{
    return toc_us(tic) /1e6;
}


// get the total time (seconds) since tic_being();
inline static double tic_total_s(tic_t * tic)
{
    return tic_total_us(tic) /1e6;
}

#endif
