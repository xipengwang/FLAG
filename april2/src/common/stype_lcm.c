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

#include <stdint.h>
#include <assert.h>
#include "stype.h"
#include "stype_lcm.h"

void stype_lcm_encode(const stype_t *stype, uint8_t *data, uint32_t *datapos, const void *obj)
{
    struct stype_lcm_impl *impl = stype->impl;

    if (data) {
        int size = impl->lcm_encode(data, *datapos, INT32_MAX, obj);
        assert(size >= 0);
        *datapos += size;
    } else {
        int size = impl->lcm_encoded_size(obj);
        *datapos += size;
    }
}

void *stype_lcm_decode(const stype_t *stype, const uint8_t *data, uint32_t *datapos, uint32_t datalen)
{
    struct stype_lcm_impl *impl = stype->impl;
    void *obj = malloc(impl->lcm_struct_size);

    int sz = impl->lcm_decode(data, *datapos, datalen, obj);
    *datapos += sz;
    return obj;
}

stype_t *stype_create_from_lcm(const char *stype_name,
                               int lcm_struct_size,
                               int (*lcm_encoded_size)(const void *obj),
                               int (*lcm_encode)(void *buf, int offset, int maxlen, const void *obj),
                               int (*lcm_decode)(const void *buf, int offset, int maxlen, void *obj))
{
    stype_t *stype = calloc(1, sizeof(stype_t));
    stype->name = strdup(stype_name);
    stype->encode = stype_lcm_encode;
    stype->decode = stype_lcm_decode;

    struct stype_lcm_impl *impl = calloc(1, sizeof(struct stype_lcm_impl));
    stype->impl = impl;
    impl->lcm_struct_size = lcm_struct_size;
    impl->lcm_encoded_size = lcm_encoded_size;
    impl->lcm_encode = lcm_encode;
    impl->lcm_decode = lcm_decode;
    return stype;
}
