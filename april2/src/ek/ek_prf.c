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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ek.h"

static void buffer_append(uint8_t **_buf, int *bufpos, const void *s, int slen)
{
    uint8_t *buf = *_buf;

    buf = realloc(buf, *bufpos + slen);
    *_buf = buf;

    memcpy(&buf[*bufpos], s, slen);
    (*bufpos) += slen;
}

// The TLS_12_PRF function except that it takes a generic hashing
// algorithm. When using SHA256, it is the official TLS_12_PRF.
static void tls_12_prf(const struct ek_prf_algorithm *ek_prf_alg,
                       const uint8_t *secret, int secret_len,
                       const char *label,
                       const uint8_t *seed, int seed_len,
                       uint8_t *out, int out_len)
{
    const ek_hash_algorithm_t *alg = &ek_prf_alg->u.tls12.hash_alg;

    const int digest_size = alg->digest_size;

    int out_pos = 0;

    // initialize A(1)
    uint8_t Ai[digest_size];
    if (1) {
        uint8_t *buf = NULL;
        int buf_len = 0;

        buffer_append(&buf, &buf_len, label, strlen(label));
        buffer_append(&buf, &buf_len, seed, seed_len);
        ek_hash_hmac(alg,
                  secret, secret_len, buf, buf_len, Ai);
        free(buf);
    }

    while (out_pos < out_len) {
        uint8_t *buf = NULL;
        int buf_len = 0;

        buffer_append(&buf, &buf_len, Ai, digest_size);
        buffer_append(&buf, &buf_len, label, strlen(label));
        buffer_append(&buf, &buf_len, seed, seed_len);

        uint8_t tmp[digest_size];
        ek_hash_hmac(alg,
                  secret, secret_len, buf, buf_len, tmp);

        for (int i = 0; i < digest_size && out_pos < out_len; i++)
            out[out_pos++] = tmp[i];

        // compute next A
        ek_hash_hmac(alg,
                  secret, secret_len, Ai, digest_size, tmp);

        memcpy(Ai, tmp, digest_size);

        free(buf);
    }
}

// returns 0 on success
int ek_prf_algorithm_get(const char *name, ek_prf_algorithm_t *alg)
{
    if (!strcmp(name, "TLS_12_SHA256")) {
        memset(alg, 0, sizeof(ek_prf_algorithm_t));

        if (ek_hash_algorithm_get("SHA256", &alg->u.tls12.hash_alg)) {
            return -1;
        }

        alg->prf = tls_12_prf;

        return 0;
    }

    return -1;
}
