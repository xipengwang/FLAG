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
#include <string.h>
#include <stdio.h>
#include <assert.h>

#include "ek_block_cipher.h"

// beware cache timing attacks. Assuming these tables are so small
// that they would be impractical.
uint8_t aes_subbyte_table[] = {
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
    0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
    0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
    0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
    0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
    0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
    0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
    0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
    0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
    0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
    0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
    0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
    0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
    0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
    0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16 };

uint8_t aes_invsubbyte_table[] = {
    0x52, 0x09, 0x6a, 0xd5, 0x30, 0x36, 0xa5, 0x38, 0xbf, 0x40, 0xa3, 0x9e, 0x81, 0xf3, 0xd7, 0xfb,
    0x7c, 0xe3, 0x39, 0x82, 0x9b, 0x2f, 0xff, 0x87, 0x34, 0x8e, 0x43, 0x44, 0xc4, 0xde, 0xe9, 0xcb,
    0x54, 0x7b, 0x94, 0x32, 0xa6, 0xc2, 0x23, 0x3d, 0xee, 0x4c, 0x95, 0x0b, 0x42, 0xfa, 0xc3, 0x4e,
    0x08, 0x2e, 0xa1, 0x66, 0x28, 0xd9, 0x24, 0xb2, 0x76, 0x5b, 0xa2, 0x49, 0x6d, 0x8b, 0xd1, 0x25,
    0x72, 0xf8, 0xf6, 0x64, 0x86, 0x68, 0x98, 0x16, 0xd4, 0xa4, 0x5c, 0xcc, 0x5d, 0x65, 0xb6, 0x92,
    0x6c, 0x70, 0x48, 0x50, 0xfd, 0xed, 0xb9, 0xda, 0x5e, 0x15, 0x46, 0x57, 0xa7, 0x8d, 0x9d, 0x84,
    0x90, 0xd8, 0xab, 0x00, 0x8c, 0xbc, 0xd3, 0x0a, 0xf7, 0xe4, 0x58, 0x05, 0xb8, 0xb3, 0x45, 0x06,
    0xd0, 0x2c, 0x1e, 0x8f, 0xca, 0x3f, 0x0f, 0x02, 0xc1, 0xaf, 0xbd, 0x03, 0x01, 0x13, 0x8a, 0x6b,
    0x3a, 0x91, 0x11, 0x41, 0x4f, 0x67, 0xdc, 0xea, 0x97, 0xf2, 0xcf, 0xce, 0xf0, 0xb4, 0xe6, 0x73,
    0x96, 0xac, 0x74, 0x22, 0xe7, 0xad, 0x35, 0x85, 0xe2, 0xf9, 0x37, 0xe8, 0x1c, 0x75, 0xdf, 0x6e,
    0x47, 0xf1, 0x1a, 0x71, 0x1d, 0x29, 0xc5, 0x89, 0x6f, 0xb7, 0x62, 0x0e, 0xaa, 0x18, 0xbe, 0x1b,
    0xfc, 0x56, 0x3e, 0x4b, 0xc6, 0xd2, 0x79, 0x20, 0x9a, 0xdb, 0xc0, 0xfe, 0x78, 0xcd, 0x5a, 0xf4,
    0x1f, 0xdd, 0xa8, 0x33, 0x88, 0x07, 0xc7, 0x31, 0xb1, 0x12, 0x10, 0x59, 0x27, 0x80, 0xec, 0x5f,
    0x60, 0x51, 0x7f, 0xa9, 0x19, 0xb5, 0x4a, 0x0d, 0x2d, 0xe5, 0x7a, 0x9f, 0x93, 0xc9, 0x9c, 0xef,
    0xa0, 0xe0, 0x3b, 0x4d, 0xae, 0x2a, 0xf5, 0xb0, 0xc8, 0xeb, 0xbb, 0x3c, 0x83, 0x53, 0x99, 0x61,
    0x17, 0x2b, 0x04, 0x7e, 0xba, 0x77, 0xd6, 0x26, 0xe1, 0x69, 0x14, 0x63, 0x55, 0x21, 0x0c, 0x7d };

static uint32_t aes_Rcon_table[] = {
    0x01000000, 0x02000000, 0x04000000, 0x08000000,
    0x10000000, 0x20000000, 0x40000000, 0x80000000,
    0x1b000000, 0x36000000, 0x6c000000, 0xd8000000,
    0xab000000, 0x4d000000, 0x9a000000};

static inline uint32_t aes_subword(uint32_t v)
{
    uint8_t v0 = aes_subbyte_table[(v >> 24) & 0xff];
    uint8_t v1 = aes_subbyte_table[(v >> 16) & 0xff];
    uint8_t v2 = aes_subbyte_table[(v >> 8) & 0xff];
    uint8_t v3 = aes_subbyte_table[(v >> 0) & 0xff];

    return (v0 << 24) | (v1 << 16) | (v2 << 8) | (v3 << 0);
}

static inline uint32_t aes_invsubword(uint32_t v)
{
    uint8_t v0 = aes_invsubbyte_table[(v >> 24) & 0xff];
    uint8_t v1 = aes_invsubbyte_table[(v >> 16) & 0xff];
    uint8_t v2 = aes_invsubbyte_table[(v >> 8) & 0xff];
    uint8_t v3 = aes_invsubbyte_table[(v >> 0) & 0xff];

    return (v0 << 24) | (v1 << 16) | (v2 << 8) | (v3 << 0);
}

static inline uint32_t aes_rotword(uint32_t v)
{
    return (v << 8) | (v >> 24);
}

static void aes_keyexp(const ek_block_cipher_t *cipher, const void *_key, ek_block_cipher_keyexp_t *keyexp)
{
    const uint8_t *key = _key;
    const int Nb = 4;                    // Nb: # of words in block (always 4)
    const int Nk = cipher->key_size / 4; // Nk: # of words in key (4, 6, or 8)
    const int Nr = 6 + Nk;               // Nr: # of rounds (10, 12, or 14)

    uint32_t *w = keyexp->u.aes.w;

    for (int i = 0; i < Nk; i++) {
        w[i] = (key[4*i] << 24) |
            (key[4*i + 1] << 16) |
            (key[4*i + 2] << 8) |
            (key[4*i + 3] << 0);
    }

    for (int i = Nk; i < Nb * (Nr + 1); i++) {
        uint32_t temp = w[i - 1];
        if ((i % Nk) == 0)
            temp = aes_subword(aes_rotword(temp)) ^ aes_Rcon_table[(i - 1) / Nk];
        else if (Nk > 6 && (i % Nk) == 4)
            temp = aes_subword(temp);

        w[i] = w[i - Nk] ^ temp;
    }

    // for each 4x4 block of bytes, compute the transpose. (This makes
    // AddRoundKey saner. Note this is a deviation from the
    // pseudo-code in FIPS 197.)
    for (int block = 0; block < Nr + 1; block++) {
        uint32_t *M = &w[4*block];
        uint32_t Mt[4]; // transpose will go here
        memset(Mt, 0, sizeof(Mt));

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                uint8_t v = (M[i] >> (8*j)) & 0xff;
                Mt[3-j] |= v << (8*(3-i));
            }
        }

        for (int i = 0; i < 4; i++)
            M[i] = Mt[i];
    }
}

void ek_aes_encrypt(const ek_block_cipher_t *cipher, const ek_block_cipher_keyexp_t *keyexp,
                        const void *_in, void *_out)
{
    const uint8_t *in = _in;
    uint8_t *out = _out;
//    const int Nb = 4;                    // Nb: # of words in block (always 4)
    const int Nk = cipher->key_size / 4; // Nk: # of words in key (4, 6, or 8)
    const int Nr = 6 + Nk;               // Nr: # of rounds (10, 12, or 14)

    uint32_t state[4];

    // Copy Input, Row major ordering
    for (int i = 0; i < 4; i++)
        state[i] = (in[i + 0] << 24) + (in[i + 4] << 16) + (in[i + 8] << 8) + in[i + 12];

    // AddRoundKey
    for (int i = 0; i < 4; i++)
        state[i] ^= keyexp->u.aes.w[i];

    for (int round = 0; round < Nr - 1; round++) {
        // SubBytes
        for (int i = 0; i < 4; i++)
            state[i] = aes_subword(state[i]);

        // ShiftRows
        for (int i = 0; i < 4; i++)
            state[i] = (state[i] << (8*i)) | (state[i] >> (32 - 8*i));

        // MixColumns
        if (1) {
            uint32_t newstate[4] = { 0, 0, 0, 0 };

            for (int col = 0; col < 4; col++) {

                for (int i = 0; i < 4; i++)
                    newstate[i] = newstate[i] << 8;

                // we'll compute the contribution of each element in
                // the state column, accumulating it in the
                // least-significant 8 bits of newstate. (We shift
                // newstate above every time we move columns.)
                for (int row = 0; row < 4; row++) {
                    uint8_t v1 = (state[row] >> (24 - 8*col)) & 0xff;
                    uint8_t v2 = v1 << 1; // compute {02}*v
                    if (v1 & 0x80)
                        v2 ^= 0x1b;       // irreducible polynomial
                    uint8_t v3 = v1 ^ v2; // compute {03}*v

                    newstate[row] ^= v2; // the {02}s are down the diagonal
                    newstate[(row+1) & 3] ^= v1;
                    newstate[(row+2) & 3] ^= v1;
                    newstate[(row+3) & 3] ^= v3;
                }
            }

            for (int i = 0; i < 4; i++)
                state[i] = newstate[i];
        }

        // AddRoundKey
        for (int i = 0; i < 4; i++)
            state[i] ^= keyexp->u.aes.w[4*(round+1) + i];
    }

    // SubBytes
    for (int i = 0; i < 4; i++)
        state[i] = aes_subword(state[i]);

    // ShiftRows
    for (int i = 1; i < 4; i++)
        state[i] = (state[i] << (8*i)) | (state[i] >> (32 - 8*i));

    // AddRoundKey
    for (int i = 0; i < 4; i++)
        state[i] ^= keyexp->u.aes.w[4*Nr + i];

    // Output, Row major ordering
    for (int i = 0; i < 4; i++) {
        out[i + 0] = (state[i] >> 24) & 0xff;
        out[i + 4] = (state[i] >> 16) & 0xff;
        out[i + 8] = (state[i] >> 8) & 0xff;
        out[i + 12] = (state[i] >> 0) & 0xff;
    }
}

static void aes_decrypt(const ek_block_cipher_t *cipher, const ek_block_cipher_keyexp_t *keyexp,
                        const void *_in, void *_out)
{
    const uint8_t *in = _in;
    uint8_t *out = _out;
//    const int Nb = 4;                    // Nb: # of words in block (always 4)
    const int Nk = cipher->key_size / 4; // Nk: # of words in key (4, 6, or 8)
    const int Nr = 6 + Nk;               // Nr: # of rounds (10, 12, or 14)

    uint32_t state[4];

    // Copy Input, Row major ordering
    for (int i = 0; i < 4; i++)
        state[i] = (in[i + 0] << 24) + (in[i + 4] << 16) + (in[i + 8] << 8) + in[i + 12];

    // InvAddRoundKey
    for (int i = 0; i < 4; i++)
        state[i] ^= keyexp->u.aes.w[4*Nr + i];

    for (int round = Nr - 2; round >= 0; round--) {

        // InvShiftRows
        for (int i = 1; i < 4; i++)
            state[i] = (state[i] >> (8*i)) | (state[i] << (32 - 8*i));

        // InvSubBytes
        for (int i = 0; i < 4; i++)
            state[i] = aes_invsubword(state[i]);

        // AddRoundKey
        for (int i = 0; i < 4; i++)
            state[i] ^= keyexp->u.aes.w[4*(round+1) + i];

        // InvMixColumns
        if (1) {
            uint32_t newstate[4] = { 0, 0, 0, 0 };

            for (int col = 0; col < 4; col++) {

                for (int i = 0; i < 4; i++)
                    newstate[i] = newstate[i] << 8;

                // we'll compute the contribution of each element in
                // the state column, accumulating it in the
                // least-significant 8 bits of newstate. (We shift
                // newstate above every time we move columns.)
                for (int row = 0; row < 4; row++) {
                    // XXX Use a lookup table? (beware cache timing attacks)
                    uint8_t v1 = (state[row] >> (24 - 8*col)) & 0xff;
                    uint8_t v2 = v1 << 1; // compute {02}*v
                    if (v1 & 0x80)
                        v2 ^= 0x1b;       // irreducible polynomial
                    uint8_t v4 = v2 << 1; // compute {04}*v
                    if (v2 & 0x80)
                        v4 ^= 0x1b;
                    uint8_t v8 = v4 << 1; // compute {08}*v
                    if (v4 & 0x80)
                        v8 ^= 0x1b;

                    uint8_t ve = v2 ^ v4 ^ v8;
                    uint8_t v9 = v1 ^ v8;
                    uint8_t vd = v1 ^ v4 ^ v8;
                    uint8_t vb = v1 ^ v2 ^ v8;

                    newstate[row] ^= ve; // the {0e}s are down the diagonal
                    newstate[(row+1) & 3] ^= v9;
                    newstate[(row+2) & 3] ^= vd;
                    newstate[(row+3) & 3] ^= vb;
                }
            }

            for (int i = 0; i < 4; i++)
                state[i] = newstate[i];
        }
    }

    // InvShiftRows
    for (int i = 1; i < 4; i++)
        state[i] = (state[i] >> (8*i)) | (state[i] << (32 - 8*i));

    // InvSubBytes
    for (int i = 0; i < 4; i++)
        state[i] = aes_invsubword(state[i]);

    // InvAddRoundKey
    for (int i = 0; i < 4; i++)
        state[i] ^= keyexp->u.aes.w[i];

    // Output, Row major ordering
    for (int i = 0; i < 4; i++) {
        out[i + 0] = (state[i] >> 24) & 0xff;
        out[i + 4] = (state[i] >> 16) & 0xff;
        out[i + 8] = (state[i] >> 8) & 0xff;
        out[i + 12] = (state[i] >> 0) & 0xff;
    }
}

// returns 0 on success
int ek_block_cipher_get(const char *name, ek_block_cipher_t *cipher)
{
    memset(cipher, 0, sizeof(ek_block_cipher_t));

    if (!strcmp(name, "AES128")) {
        cipher->key_size = 16;
        cipher->block_size = 16;
        cipher->keyexp = aes_keyexp;
        cipher->encrypt = ek_aes_encrypt;
        cipher->decrypt = aes_decrypt;
        return 0;
    }

    if (!strcmp(name, "AES192")) {
        cipher->key_size = 24;
        cipher->block_size = 16;
        cipher->keyexp = aes_keyexp;
        cipher->encrypt = ek_aes_encrypt;
        cipher->decrypt = aes_decrypt;
        return 0;
    }

    if (!strcmp(name, "AES256")) {
        cipher->key_size = 32;
        cipher->block_size = 16;
        cipher->keyexp = aes_keyexp;
        cipher->encrypt = ek_aes_encrypt;
        cipher->decrypt = aes_decrypt;
        return 0;
    }

    return -1;
}

// NB: modifies data in-place
void ek_block_cipher_encrypt_cbc(const ek_block_cipher_t *cipher,
                                 const ek_block_cipher_keyexp_t *keyexp,
                                 const void *iv,
                                 void *_data, int len)
{
    uint8_t *data = _data;

    const int block_size = cipher->block_size;
    assert((len % block_size) == 0);

    const uint8_t *fuzzer = iv;

    for (int offset = 0; offset < len; offset += block_size) {

        for (int i = 0; i < block_size; i++)
            data[offset + i] ^= fuzzer[i];

        cipher->encrypt(cipher, keyexp, &data[offset], &data[offset]);
        fuzzer = &data[offset];
    }
}

// NB: modifies data in-place
void ek_block_cipher_decrypt_cbc(const ek_block_cipher_t *cipher,
                                 const ek_block_cipher_keyexp_t *keyexp,
                                 const void *iv,
                                 void *_data, int len)
{
    uint8_t *data = _data;

    const int block_size = cipher->block_size;
    assert((len % block_size) == 0);

    for (int offset = len - block_size; offset >= 0; offset -= block_size) {

        cipher->decrypt(cipher, keyexp, &data[offset], &data[offset]);

        const uint8_t *fuzzer = (offset == 0) ? iv : &data[offset - block_size];

        for (int i = 0; i < block_size; i++)
            data[offset + i] ^= fuzzer[i];
    }
}
