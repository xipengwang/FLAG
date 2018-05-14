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

#ifndef _CRYPTO_H
#define _CRYPTO_H

#include <stdint.h>

// Detect architecture
#if INTPTR_MAX == INT64_MAX
  #define EK_BIGINT_TYPE uint64_t
#elif INTPTR_MAX == INT32_MAX
  #define ARCH_32BIT
  #define EK_BIGINT_TYPE uint32_t
#else
  #error Unknown pointer size, or missing size macros!
#endif

typedef struct ek_bigint ek_bigint_t;
struct ek_bigint
{
    int len;
    int last; // what is the index of the last non-zero element? (always at least zero)
    EK_BIGINT_TYPE *data; // data[0] is least significant word.
};

ek_bigint_t *ek_bigint_create_int(uint64_t v);
ek_bigint_t *ek_bigint_create_nwords(int nwords);
ek_bigint_t *ek_bigint_create_nbits(int nbits);
ek_bigint_t *ek_bigint_create_from_ascii(const char *s);
ek_bigint_t *ek_bigint_create_from_bytes(const uint8_t *tmp, int tmplen);
void ek_bigint_ensure_nwords(ek_bigint_t *a, int nwords);

ek_bigint_t *ek_bigint_copy_nbits(const ek_bigint_t *a, int nbits);

void ek_bigint_set_byte(ek_bigint_t *a, int byte_idx, uint8_t v);
uint8_t ek_bigint_get_byte(const ek_bigint_t *a, int byte_idx);

// convert hex characters in ascii, most significant digit first, to a crypto_int.
int crypto_ascii_to_bytes(const char *s, uint8_t **pdata, int *plen);

char *ek_bigint_to_string_hex(ek_bigint_t *a);

void ek_bigint_destroy(ek_bigint_t *a);
void ek_bigint_print(const ek_bigint_t *a);
ek_bigint_t *ek_bigint_copy(const ek_bigint_t *a);
ek_bigint_t *ek_bigint_copy_pad(const ek_bigint_t *a, int newlen);
int ek_bigint_equal(const ek_bigint_t *a, const ek_bigint_t *b);
int ek_bigint_less_than(const ek_bigint_t *a, const ek_bigint_t *b);
int ek_bigint_less_equal(const ek_bigint_t *a, const ek_bigint_t *b);
int ek_bigint_is_zero(const ek_bigint_t *a);

void ek_bigint_mul_2_inplace(ek_bigint_t *a);
void ek_bigint_div_2_inplace(ek_bigint_t *a);
void ek_bigint_sub_inplace(ek_bigint_t *a, const ek_bigint_t *b);
void ek_bigint_add_inplace(ek_bigint_t *a, const ek_bigint_t *b);

ek_bigint_t *ek_bigint_mod(const ek_bigint_t *v, const ek_bigint_t *m);
ek_bigint_t *ek_bigint_mul(const ek_bigint_t *a, const ek_bigint_t *b);
ek_bigint_t *ek_bigint_mul_mod(const ek_bigint_t *a, const ek_bigint_t *b, const ek_bigint_t *mod);
ek_bigint_t *ek_bigint_pow_mod(const ek_bigint_t *a, const ek_bigint_t *p, const ek_bigint_t *m);


// how many bits (minimum) would be required to represent this number?
int ek_bigint_nbits(const ek_bigint_t *a);

/////////////////////////////////////////////
// Specialized code for doing mul_mod operations more quickly by
// precomputing a table of residues
struct ek_bigint_mul_lookup
{
    int tablelen;
    ek_bigint_t **table;
};

struct ek_bigint_mul_lookup *ek_bigint_mul_lookup_create(const ek_bigint_t *mod, int tablelen);
void ek_bigint_mul_lookup_destroy(struct ek_bigint_mul_lookup *lookup);

ek_bigint_t *ek_bigint_mul_mod_lookup(const ek_bigint_t *a, const ek_bigint_t *b,
                                      const struct ek_bigint_mul_lookup *lookup, const ek_bigint_t *mod, int weak);

#endif
