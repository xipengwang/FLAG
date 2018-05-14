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

#ifndef _EK_HASH_H
#define _EK_HASH_H

#include <stdint.h>

typedef struct ek_hash_state ek_hash_state_t;
struct ek_hash_state
{
    int type;

    union {
        struct {
            uint32_t w[16];
            int inpos; // where are we in current block?

            uint32_t h[5];
            uint64_t len; // total bytes of input
        } sha1;

        struct {
            uint32_t w[16];
            int inpos; // where are we in current block?

            uint32_t h[8];
            uint64_t len; // total bytes of input
        } sha2;

        struct {
            uint32_t w[16];
            int inpos;

            uint32_t h[4];
            uint64_t len;
        } md5;
    } u;
};

typedef struct ek_hash_algorithm ek_hash_algorithm_t;
struct ek_hash_algorithm
{
    const char *name;
    int digest_size;
    int block_size;

    void (*init)(ek_hash_state_t  *state);
    void (*update)(ek_hash_state_t *state, const void *buffer, int len);
    void (*final)(ek_hash_state_t *state, void *digest);
};

int ek_hash_algorithm_get(const char *name, ek_hash_algorithm_t *alg);

typedef struct ek_hmac_state ek_hmac_state_t;
struct ek_hmac_state
{
    ek_hash_algorithm_t alg;
    ek_hash_state_t inner_state;
    ek_hash_state_t outer_state;
};

void ek_hmac_init(ek_hmac_state_t *state, const ek_hash_algorithm_t *alg, const void *key, int key_len);
void ek_hmac_update(ek_hmac_state_t *state, const void *buffer, int len);
void ek_hmac_final(ek_hmac_state_t *state, void *out);

// out must have 'alg->digest_size' bytes available.
void ek_hash_hmac(const ek_hash_algorithm_t *alg,
                  const void *key, int key_len, const void *msg, int msg_len,
                  void *out);

#endif
