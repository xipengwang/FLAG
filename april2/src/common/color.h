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

#ifndef _COLOR_H
#define _COLOR_H

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

/* A utility "class" to help specifying colors */

#ifdef __cplusplus
extern "C" {
#endif

inline static uint32_t color_rgba_u32(uint8_t r, uint8_t g, uint8_t b, uint8_t a)
{
    return (((uint32_t)r) << 24) | (((uint32_t)g) << 16) |
        (((uint32_t)b) << 8) | (((uint32_t)a) << 0);
}

// attempts to use 'mem'. If NULL, we alloc mem that the user is responsible to free
inline static float *color_rgba_f4(uint8_t r, uint8_t g, uint8_t b, uint8_t a, float *mem)
{
    if(mem == NULL) mem = (float *) malloc (4*sizeof(float));
    mem[0] = (float)r / 255.0f;
    mem[1] = (float)g / 255.0f;
    mem[2] = (float)b / 255.0f;
    mem[3] = (float)a / 255.0f;
    return mem;
}

void HSVtoRGB( float *r, float *g, float *b, float h, float s, float v );

#ifdef __cplusplus
}
#endif

#endif
