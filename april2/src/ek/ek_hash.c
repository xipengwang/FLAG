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

#include <assert.h>
#include <string.h>

#include "ek_hash.h"

#define SHA1_TYPE 0x8171ffad
#define SHA2_TYPE 0x75717dde
#define MD5_TYPE  0xa5543ab0

static inline uint32_t ROTL_U32(uint32_t v, int amt)
{
    assert(amt > 0);
    return (v << amt) | (v >> (32 - amt));
}

static inline uint32_t ROTR_U32(uint32_t v, int amt)
{
    assert(amt > 0);
    return (v >> amt) | (v << (32 - amt));
}

static inline uint32_t SHIFTR_U32(uint32_t v, int amt)
{
    assert(amt > 0);
    return (v >> amt);
}

///////////////////////////////////////////////////////////////
// MD5

static void md5_init(ek_hash_state_t *state)
{
    memset(state, 0, sizeof(ek_hash_state_t));

    state->type = MD5_TYPE;
    state->u.md5.h[0] = 0x67452301;
    state->u.md5.h[1] = 0xefcdab89;
    state->u.md5.h[2] = 0x98badcfe;
    state->u.md5.h[3] = 0x10325476;
}

const uint32_t md5_shift[] = { 7, 12, 17, 22,  7, 12, 17, 22,  7, 12, 17, 22,  7, 12, 17, 22,
                               5,  9, 14, 20,  5,  9, 14, 20,  5,  9, 14, 20,  5,  9, 14, 20,
                               4, 11, 16, 23,  4, 11, 16, 23,  4, 11, 16, 23,  4, 11, 16, 23,
                               6, 10, 15, 21,  6, 10, 15, 21,  6, 10, 15, 21,  6, 10, 15, 21 };

const uint32_t md5_k[] = {  0xd76aa478, 0xe8c7b756, 0x242070db, 0xc1bdceee,
                            0xf57c0faf, 0x4787c62a, 0xa8304613, 0xfd469501,
                            0x698098d8, 0x8b44f7af, 0xffff5bb1, 0x895cd7be,
                            0x6b901122, 0xfd987193, 0xa679438e, 0x49b40821,
                            0xf61e2562, 0xc040b340, 0x265e5a51, 0xe9b6c7aa,
                            0xd62f105d, 0x02441453, 0xd8a1e681, 0xe7d3fbc8,
                            0x21e1cde6, 0xc33707d6, 0xf4d50d87, 0x455a14ed,
                            0xa9e3e905, 0xfcefa3f8, 0x676f02d9, 0x8d2a4c8a,
                            0xfffa3942, 0x8771f681, 0x6d9d6122, 0xfde5380c,
                            0xa4beea44, 0x4bdecfa9, 0xf6bb4b60, 0xbebfbc70,
                            0x289b7ec6, 0xeaa127fa, 0xd4ef3085, 0x04881d05,
                            0xd9d4d039, 0xe6db99e5, 0x1fa27cf8, 0xc4ac5665,
                            0xf4292244, 0x432aff97, 0xab9423a7, 0xfc93a039,
                            0x655b59c3, 0x8f0ccc92, 0xffeff47d, 0x85845dd1,
                            0x6fa87e4f, 0xfe2ce6e0, 0xa3014314, 0x4e0811a1,
                            0xf7537e82, 0xbd3af235, 0x2ad7d2bb, 0xeb86d391 };

static void md5_update_one(ek_hash_state_t *state, uint8_t v)
{
    int wordidx = state->u.md5.inpos / 4;
    int wordpos = state->u.md5.inpos & 3;
    int bitpos = 8 * (wordpos);

    state->u.md5.w[wordidx] |= (v << bitpos);
    state->u.md5.inpos++;
    state->u.md5.len++;

    if (state->u.md5.inpos < 64)
        return;

    uint32_t a = state->u.md5.h[0];
    uint32_t b = state->u.md5.h[1];
    uint32_t c = state->u.md5.h[2];
    uint32_t d = state->u.md5.h[3];

    for (int i = 0; i < 64; i++) {
        uint32_t f, g;

        if (i < 16) {
            f = (b & c) | ((~b) & d);
            g = i;
        } else if (i < 32) {
            f = (d & b) | ((~d) & c);
            g = (5*i + 1) & 15;
        } else if (i < 48) {
            f = b ^ c ^ d;
            g = (3*i + 5) & 15;
        } else {
            f = c ^ (b | (~d));
            g = (7*i) & 15;
        }
        uint32_t dtmp = d;
        d = c;
        c = b;
        b = b + ROTL_U32(a + f + md5_k[i] + state->u.md5.w[g], md5_shift[i]);
        a = dtmp;
    }

    state->u.md5.h[0] += a;
    state->u.md5.h[1] += b;
    state->u.md5.h[2] += c;
    state->u.md5.h[3] += d;

    state->u.md5.inpos = 0;

    for (int i = 0; i < 16; i++)
        state->u.md5.w[i] = 0;
}

static void md5_update(ek_hash_state_t *state, const void *_buffer, int len)
{
    const uint8_t *buffer = _buffer;

    for (int i = 0; i < len; i++)
        md5_update_one(state, buffer[i]);
}

static void md5_final(ek_hash_state_t *state, void *_digest)
{
    uint8_t *digest = _digest;
    uint64_t nbits = 8 * state->u.md5.len;

    md5_update_one(state, 0x80);
    while (state->u.md5.inpos != 56)
        md5_update_one(state, 0x00);

    for (int i = 0; i < 8; i++)
        md5_update_one(state, (nbits >> (8 * (i))) & 0xff);

    assert(state->u.md5.inpos == 0);

    for (int i = 0; i < 16; i++) {
        int wordidx = i / 4;
        int wordpos = i & 3;
        int bitpos = 8 * (wordpos);

        digest[i] = (state->u.md5.h[wordidx] >> bitpos) & 0xff;
    }
}

///////////////////////////////////////////////////////////////
// SHA1 (160 bits)

static void sha1_init(ek_hash_state_t *state)
{
    memset(state, 0, sizeof(ek_hash_state_t));

    state->type = SHA1_TYPE;
    state->u.sha1.h[0] = 0x67452301;
    state->u.sha1.h[1] = 0xEFCDAB89;
    state->u.sha1.h[2] = 0x98BADCFE;
    state->u.sha1.h[3] = 0x10325476;
    state->u.sha1.h[4] = 0xC3D2E1F0;
}

static void sha1_update_one(ek_hash_state_t *state, uint8_t v)
{
    int wordidx = state->u.sha1.inpos / 4;
    int wordpos = state->u.sha1.inpos & 3;
    int bitpos = 8 * (3 - wordpos);

    state->u.sha1.w[wordidx] |= (v << bitpos);
    state->u.sha1.inpos++;
    state->u.sha1.len++;

    if (state->u.sha1.inpos < 64)
        return;

    uint32_t w[80];
    for (int i = 0; i < 16; i++)
        w[i] = state->u.sha1.w[i];
    for (int i = 16; i < 80; i++)
        w[i] = ROTL_U32(w[i-3] ^ w[i-8] ^ w[i-14] ^ w[i-16], 1);

    uint32_t a = state->u.sha1.h[0];
    uint32_t b = state->u.sha1.h[1];
    uint32_t c = state->u.sha1.h[2];
    uint32_t d = state->u.sha1.h[3];
    uint32_t e = state->u.sha1.h[4];

    uint32_t f, k;

    for (int i = 0; i < 80; i++) {
        if (i < 20) {
            f = (b & c) | ((~b) & d);
            k = 0x5A827999;
        } else if (i < 40) {
            f = b ^ c ^ d;
            k = 0x6ED9EBA1;
        } else if (i < 60) {
            f = (b & c) | (b & d) | (c & d);
            k = 0x8F1BBCDC;
        } else {
            f = b ^ c ^ d;
            k = 0xCA62C1D6;
        }

        uint32_t tmp = ROTL_U32(a, 5) + f + e + k + w[i];
        e = d;
        d = c;
        c = ROTL_U32(b, 30);
        b = a;
        a = tmp;
    }

    state->u.sha1.h[0] += a;
    state->u.sha1.h[1] += b;
    state->u.sha1.h[2] += c;
    state->u.sha1.h[3] += d;
    state->u.sha1.h[4] += e;

    for (int i = 0; i < 16; i++)
        state->u.sha1.w[i] = 0;

    state->u.sha1.inpos = 0;
}

static void sha1_update(ek_hash_state_t *state, const void *_buffer, int len)
{
    const uint8_t *buffer = _buffer;

    for (int i = 0; i < len; i++)
        sha1_update_one(state, buffer[i]);
}

static void sha1_final(ek_hash_state_t *state, void *_digest)
{
    uint8_t *digest = _digest;
    uint64_t nbits = 8 * state->u.sha1.len;

    sha1_update_one(state, 0x80);
    while (state->u.sha1.inpos != 56)
        sha1_update_one(state, 0x00);

    for (int i = 0; i < 8; i++)
        sha1_update_one(state, (nbits >> (8 * (7 - i))) & 0xff);

    assert(state->u.sha1.inpos == 0);

    for (int i = 0; i < 20; i++) {
        int wordidx = i / 4;
        int wordpos = i & 3;
        int bitpos = 8 * (3 - wordpos);

        digest[i] = (state->u.sha1.h[wordidx] >> bitpos) & 0xff;
    }
}

///////////////////////////////////////////////////////////////
// SHA2
static void sha2_init(ek_hash_state_t *state)
{
    memset(state, 0, sizeof(ek_hash_state_t));

    state->type = SHA2_TYPE;
    state->u.sha2.h[0] = 0x6a09e667;
    state->u.sha2.h[1] = 0xbb67ae85;
    state->u.sha2.h[2] = 0x3c6ef372;
    state->u.sha2.h[3] = 0xa54ff53a;
    state->u.sha2.h[4] = 0x510e527f;
    state->u.sha2.h[5] = 0x9b05688c;
    state->u.sha2.h[6] = 0x1f83d9ab;
    state->u.sha2.h[7] = 0x5be0cd19;
}

const uint32_t SHA2_K[] = { 0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
                            0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
                            0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
                            0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
                            0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
                            0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
                            0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
                            0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
                            0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
                            0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
                            0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
                            0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
                            0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
                            0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
                            0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
                            0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2 };

static void sha2_update_one(ek_hash_state_t *state, uint8_t v)
{
    int wordidx = state->u.sha2.inpos / 4;
    int wordpos = state->u.sha2.inpos & 3;
    int bitpos = 8 * (3 - wordpos);

    state->u.sha2.w[wordidx] |= (v << bitpos);
    state->u.sha2.inpos++;
    state->u.sha2.len++;

    if (state->u.sha2.inpos < 64)
        return;

    uint32_t w[64];
    for (int i = 0; i < 16; i++)
        w[i] = state->u.sha2.w[i];

    for (int i = 16; i < 64; i++) {
        uint32_t s0 = ROTR_U32(w[i-15], 7) ^ ROTR_U32(w[i-15], 18) ^ SHIFTR_U32(w[i-15], 3);
        uint32_t s1 = ROTR_U32(w[i-2], 17) ^ ROTR_U32(w[i-2], 19) ^ SHIFTR_U32(w[i-2], 10);

        w[i] = w[i-16] + s0 + w[i-7] + s1;
    }

    uint32_t a = state->u.sha2.h[0];
    uint32_t b = state->u.sha2.h[1];
    uint32_t c = state->u.sha2.h[2];
    uint32_t d = state->u.sha2.h[3];
    uint32_t e = state->u.sha2.h[4];
    uint32_t f = state->u.sha2.h[5];
    uint32_t g = state->u.sha2.h[6];
    uint32_t h = state->u.sha2.h[7];

    for (int i = 0; i < 64; i++) {
        uint32_t s1 = ROTR_U32(e, 6) ^ ROTR_U32(e, 11) ^ ROTR_U32(e, 25);
        uint32_t ch = (e & f) ^ ((~e) & g);
        uint32_t tmp1 = h + s1 + ch + SHA2_K[i] + w[i];
        uint32_t s0 = ROTR_U32(a, 2) ^ ROTR_U32(a, 13) ^ ROTR_U32(a, 22);
        uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
        uint32_t tmp2 = s0 + maj;

        h = g;
        g = f;
        f = e;
        e = d + tmp1;
        d = c;
        c = b;
        b = a;
        a = tmp1 + tmp2;
    }

    state->u.sha2.h[0] += a;
    state->u.sha2.h[1] += b;
    state->u.sha2.h[2] += c;
    state->u.sha2.h[3] += d;
    state->u.sha2.h[4] += e;
    state->u.sha2.h[5] += f;
    state->u.sha2.h[6] += g;
    state->u.sha2.h[7] += h;

    for (int i = 0; i < 16; i++)
        state->u.sha2.w[i] = 0;

    state->u.sha2.inpos = 0;
}

static void sha2_update(ek_hash_state_t *state, const void *_buffer, int len)
{
    const uint8_t *buffer = _buffer;

    for (int i = 0; i < len; i++)
        sha2_update_one(state, buffer[i]);
}

static void sha2_final(ek_hash_state_t *state, void *_digest)
{
    uint8_t *digest = _digest;
    uint64_t nbits = 8 * state->u.sha2.len;

    sha2_update_one(state, 0x80);
    while (state->u.sha2.inpos != 56)
        sha2_update_one(state, 0x00);

    for (int i = 0; i < 8; i++)
        sha2_update_one(state, (nbits >> (8 * (7 - i))) & 0xff);

    assert(state->u.sha2.inpos == 0);

    for (int i = 0; i < 32; i++) {
        int wordidx = i / 4;
        int wordpos = i & 3;
        int bitpos = 8 * (3 - wordpos);

        digest[i] = (state->u.sha2.h[wordidx] >> bitpos) & 0xff;
    }
}

///////////////////////////////////////////////////////////////
// Utility

// returns 0 on success.
int ek_hash_algorithm_get(const char *name, ek_hash_algorithm_t *alg)
{
    if (!strcmp(name, "SHA1")) {
        alg->name = "SHA1";
        alg->digest_size = 20;
        alg->block_size = 64;
        alg->init = sha1_init;
        alg->update = sha1_update;
        alg->final = sha1_final;
        return 0;
    }

    if (!strcmp(name, "SHA256")) {
        alg->name = "SHA256";
        alg->digest_size = 32;
        alg->block_size = 64;
        alg->init = sha2_init;
        alg->update = sha2_update;
        alg->final = sha2_final;
        return 0;
    }

    if (!strcmp(name, "MD5")) {
        alg->name = "MD5";
        alg->digest_size = 16;
        alg->block_size = 64;
        alg->init = md5_init;
        alg->update = md5_update;
        alg->final = md5_final;
        return 0;
    }

    return -1;
}

void ek_hmac_init(ek_hmac_state_t *state, const ek_hash_algorithm_t *alg, const void *key, int key_len)
{
    memset(state, 0, sizeof(ek_hmac_state_t));
    state->alg = *alg;
    state->alg.init(&state->inner_state);
    state->alg.init(&state->outer_state);

    uint8_t key_pad[alg->block_size];
    memset(key_pad, 0, alg->block_size);

    if (key_len > alg->block_size) { // shorten longer keys
        ek_hash_state_t state;
        alg->init(&state);
        alg->update(&state, key, key_len);
        alg->final(&state, key_pad);
    } else {
        // pad shorter keys (just copies if key was right length)
        memcpy(key_pad, key, key_len);
        for (int i = key_len; i < alg->block_size; i++)
            key_pad[i] = 0;
    }

    for (int i = 0; i < alg->block_size; i++)
        key_pad[i] ^= 0x36;

    state->alg.update(&state->inner_state, key_pad, alg->block_size);

    for (int i = 0; i < alg->block_size; i++)
        key_pad[i] ^= (0x36 ^ 0x5c); // undo 0x36 above.

    state->alg.update(&state->outer_state, key_pad, alg->block_size);
}

void ek_hmac_update(ek_hmac_state_t *state, const void *buffer, int len)
{
    state->alg.update(&state->inner_state, buffer, len);
}

void ek_hmac_final(ek_hmac_state_t *state, void *out)
{
    uint8_t inner[state->alg.digest_size];

    state->alg.final(&state->inner_state, inner);
    state->alg.update(&state->outer_state, inner, state->alg.digest_size);
    state->alg.final(&state->outer_state, out);
}


// out must have 'alg->digest_size' bytes available.
void ek_hash_hmac(const ek_hash_algorithm_t *alg,
                  const void *key, int key_len, const void *msg, int msg_len,
                  void *out)
{
    ek_hmac_state_t state;
    ek_hmac_init(&state, alg, key, key_len);
    ek_hmac_update(&state, msg, msg_len);
    ek_hmac_final(&state, out);
}
