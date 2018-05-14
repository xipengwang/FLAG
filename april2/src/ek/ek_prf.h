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

#ifndef _EK_PRF_H
#define _EK_PRF_H

#include "ek.h"

typedef struct ek_prf_algorithm ek_prf_algorithm_t;
struct ek_prf_algorithm
{
    void (*prf)(const struct ek_prf_algorithm *alg,
                const uint8_t *secret, int secret_len,
                const char *label,
                const uint8_t *seed, int seed_len,
                uint8_t *out, int out_len);

    union {
        struct {
            ek_hash_algorithm_t hash_alg;
        } tls12;
    } u;
};

int ek_prf_algorithm_get(const char *name, ek_prf_algorithm_t *alg);

#endif
