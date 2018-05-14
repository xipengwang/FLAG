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

#ifndef _EK_BLOCK_CIPHER_H
#define _EK_BLOCK_CIPHER_H

#include <stdint.h>

typedef struct ek_block_cipher_keyexp ek_block_cipher_keyexp_t;
struct ek_block_cipher_keyexp
{
    union {
        struct {
            uint32_t w[60]; // worst case for 256 bit keys
        } aes;
    } u;
};

typedef struct ek_block_cipher ek_block_cipher_t;
struct ek_block_cipher
{
    int block_size; // in bytes
    int key_size; // in bytes

    void (*keyexp)(const ek_block_cipher_t *cipher, const void *key, ek_block_cipher_keyexp_t *keyexp);
    void (*encrypt)(const ek_block_cipher_t *cipher, const ek_block_cipher_keyexp_t *keyexp, const void *in, void *out);
    void (*decrypt)(const ek_block_cipher_t *cipher, const ek_block_cipher_keyexp_t *keyexp, const void *in, void *out);
};

int ek_block_cipher_get(const char *name, ek_block_cipher_t *cipher);

//void ek_aes_encrypt(const ek_block_cipher_t *cipher, const ek_block_cipher_keyexp_t *keyexp,
//                    const void *_in, void *_out);

void ek_block_cipher_encrypt_cbc(const ek_block_cipher_t *cipher,
                                 const ek_block_cipher_keyexp_t *keyexp,
                                 const void *iv,
                                 void *_data, int len);

void ek_block_cipher_decrypt_cbc(const ek_block_cipher_t *cipher,
                                 const ek_block_cipher_keyexp_t *keyexp,
                                 const void *iv,
                                 void *_data, int len);

#endif
