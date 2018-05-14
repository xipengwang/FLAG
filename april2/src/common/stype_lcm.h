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

#ifndef _STYPE_LCM_H
#define _STYPE_LCM_H

#include "common/stype.h"

/* A convenience macro and support code that makes it easy to create stypes from LCM declarations.

   Typical usage:

       stype_register(STYPE_FROM_LCM("laser_t", laser_t));

 */
struct stype_lcm_impl
{
    int lcm_struct_size;
    int (*lcm_encode)(void *buf, int offset, int maxlen, const void *p);
    int (*lcm_encoded_size)(const void *obj);
    int (*lcm_decode)(const void *buf, int offset, int maxlen, void *obj);
};

#define STYPE_FROM_LCM(lcmname, lcmtype) stype_create_from_lcm(lcmname, sizeof(lcmtype), (void*) lcmtype ## _encoded_size, (void*) lcmtype ## _encode, (void*) lcmtype ## _decode)

void stype_lcm_encode(const stype_t *stype, uint8_t *data, uint32_t *datapos, const void *obj);
void *stype_lcm_decode(const stype_t *stype, const uint8_t *data, uint32_t *datapos, uint32_t datalen);

stype_t *stype_create_from_lcm(const char *stype_name,
                               int lcm_struct_size,
                               int (*lcm_encoded_size)(const void *obj),
                               int (*lcm_encode)(void *buf, int offset, int maxlen, const void *obj),
                               int (*lcm_decode)(const void *buf, int offset, int maxlen, void *obj));

#endif
