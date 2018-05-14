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
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>

#include "ek.h"

static inline int imax(int a, int b)
{
    return a > b ? a : b;
}

static inline void ek_bigint_fix_last(ek_bigint_t *a)
{
    a->last = 0;
    for (int i = 0; i < a->len; i++)
        if (a->data[i])
            a->last = i;
}

static inline void ek_bigint_validate(const ek_bigint_t *a)
{
    if (0) {
        int last = 0;
        for (int i = 0; i < a->len; i++)
            if (a->data[i])
                last = i;

        assert(last == a->last);
    }
}

ek_bigint_t *ek_bigint_create_int(uint64_t v)
{
    ek_bigint_t *a = ek_bigint_create_nbits(64);
    for (int i = 0; i < 8; i++)
        ek_bigint_set_byte(a, i, (v >> (8*i)) & 0xff);
    return a;
}

ek_bigint_t *ek_bigint_create_nwords(int nwords)
{
    ek_bigint_t *a = calloc(1, sizeof(ek_bigint_t));
    a->len = nwords;
    a->data = calloc(sizeof(EK_BIGINT_TYPE), a->len);

    return a;
}

ek_bigint_t *ek_bigint_create_nbits(int nbits)
{
    return ek_bigint_create_nwords((nbits + 8*sizeof(EK_BIGINT_TYPE) - 1) / (8 * sizeof(EK_BIGINT_TYPE)));
}

// byte_idx = 0 corresponds to the least significant byte.
void ek_bigint_set_byte(ek_bigint_t *a, int byte_idx, uint8_t v)
{
    ek_bigint_validate(a);

    int idx = byte_idx / sizeof(EK_BIGINT_TYPE);
    int b = byte_idx % sizeof(EK_BIGINT_TYPE);
    EK_BIGINT_TYPE mask = ((EK_BIGINT_TYPE) 0xff) << (8*b);

    ek_bigint_ensure_nwords(a, idx + 1);

    a->data[idx] &= (~mask);
    a->data[idx] |= (((EK_BIGINT_TYPE) v) << (8*b));

    if (v) {
        if (idx > a->last)
            a->last = idx;
    } else {
        ek_bigint_fix_last(a);
    }
}

uint8_t ek_bigint_get_byte(const ek_bigint_t *a, int byte_idx)
{
    ek_bigint_validate(a);

    int idx = byte_idx / sizeof(EK_BIGINT_TYPE);
    int b = byte_idx % sizeof(EK_BIGINT_TYPE);

    if (idx > a->last)
        return 0;

    return (a->data[idx] >> (b*8)) & 0xff;
}

// convert hex characters in ascii, most significant digit first, to a crypto_int.
ek_bigint_t *ek_bigint_create_from_ascii(const char *s)
{
    uint8_t *tmp;
    int tmplen;

    if (ek_ascii_to_bytes(s, &tmp, &tmplen))
        return NULL;

    // reverse order so that the first hex digits were the most significant
    ek_bigint_t *a = ek_bigint_create_int(0); // _nbits(tmplen * 8);
    for (int i = 0; i < tmplen; i++) {
        ek_bigint_set_byte(a, i, tmp[tmplen - 1 - i]);
    }

    free(tmp);

    ek_bigint_fix_last(a);
    return a;
}

// assumed MSB is at index 0.
ek_bigint_t *ek_bigint_create_from_bytes(const uint8_t *tmp, int tmplen)
{
    // reverse order so that the first hex digits were the most significant
    ek_bigint_t *a = ek_bigint_create_int(0);
    for (int i = 0; i < tmplen; i++) {
        ek_bigint_set_byte(a, i, tmp[tmplen - 1 - i]);
    }

    ek_bigint_fix_last(a);
    return a;
}

void ek_bigint_destroy(ek_bigint_t *a)
{
    if (!a)
        return;

    ek_bigint_validate(a);

    if (!a)
        return;
    memset(a->data, 0, sizeof(EK_BIGINT_TYPE) *  a->len);
    free(a->data);
    free(a);
}

void ek_bigint_print(const ek_bigint_t *a)
{
    ek_bigint_validate(a);

    int nbytes = sizeof(EK_BIGINT_TYPE)*a->len;
    int nz = 0;
    for (int i = nbytes - 1; i >= 0; i--) {
        int v = ek_bigint_get_byte(a, i);
        nz |= v;
        if (nz)
            printf("%02X", v);
    }

    if (!nz)
        printf("0");

    printf("\n");
}

ek_bigint_t *ek_bigint_copy(const ek_bigint_t *a)
{
    ek_bigint_validate(a);

    ek_bigint_t *r = calloc(1, sizeof(ek_bigint_t));
    r->len = a->len;
    r->last = a->last;
    r->data = calloc(1, sizeof(EK_BIGINT_TYPE) * r->len);
    memcpy(r->data, a->data, sizeof(EK_BIGINT_TYPE) * r->len);

    ek_bigint_validate(r);
    return r;
}

ek_bigint_t *ek_bigint_copy_nwords(const ek_bigint_t *a, int nwords)
{
    ek_bigint_validate(a);

    ek_bigint_t *r = calloc(1, sizeof(ek_bigint_t));
    r->len = nwords;
    r->data = calloc(1, sizeof(EK_BIGINT_TYPE) * r->len);
    memcpy(r->data, a->data, sizeof(EK_BIGINT_TYPE) * (r->len < a->len ? r->len : a->len));

    ek_bigint_fix_last(r);
    return r;
}

void ek_bigint_ensure_nwords(ek_bigint_t *a, int nwords)
{
    if (a->len < nwords) {
        a->data = realloc(a->data, sizeof(EK_BIGINT_TYPE) * nwords);
        for (int i = a->len; i < nwords; i++)
            a->data[i] = 0;
        a->len = nwords;
    }
}

// ensure that there are at least 'nbits' of precision available in the int.
ek_bigint_t *ek_bigint_copy_nbits(const ek_bigint_t *a, int nbits)
{
    assert(nbits >= ek_bigint_nbits(a));
    ek_bigint_validate(a);

    ek_bigint_t *r = calloc(1, sizeof(ek_bigint_t));
    r->len = (nbits + 8*sizeof(EK_BIGINT_TYPE) - 1) / (8 * sizeof(EK_BIGINT_TYPE));
    r->data = calloc(1, sizeof(EK_BIGINT_TYPE) * r->len);
    memcpy(r->data, a->data, sizeof(EK_BIGINT_TYPE) * (r->len < a->len ? r->len : a->len));
    ek_bigint_fix_last(r);
    return r;
}

int ek_bigint_equal(const ek_bigint_t *a, const ek_bigint_t *b)
{
    ek_bigint_validate(a);
    ek_bigint_validate(b);

    if (a->last != b->last)
        return 0;

    for (int i = a->last; i >= 0; i--) {
        if (a->data[i] != b->data[i]) {
            return 0;
        }
    }

    // equal!
    return 1;
}

int ek_bigint_less_than(const ek_bigint_t *a, const ek_bigint_t *b)
{
    ek_bigint_validate(a);
    ek_bigint_validate(b);

    if (a->last < b->last)
        return 1;

    if (a->last > b->last)
        return 0;

    assert(a->last == b->last);

    for (int i = a->last; i >= 0; i--) {
        if (a->data[i] > b->data[i])
            return 0;
        else if (a->data[i] < b->data[i])
            return 1;
        else
            continue; // can't tell yet!
    }

    // equal!
    return 0;
}

int ek_bigint_less_equal(const ek_bigint_t *a, const ek_bigint_t *b)
{
    ek_bigint_validate(a);
    ek_bigint_validate(b);

    if (a->last < b->last)
        return 1;

    if (a->last > b->last)
        return 0;

    assert(a->last == b->last);

    for (int i = a->last; i >= 0; i--) {

        if (a->data[i] > b->data[i])
            return 0;
        else if (a->data[i] < b->data[i])
            return 1;
        else
            continue; // can't tell yet!
    }

    // equal
    return 1;
}

void ek_bigint_mul_2_inplace(ek_bigint_t *a)
{
    ek_bigint_validate(a);

    ek_bigint_ensure_nwords(a, a->last + 2);

    EK_BIGINT_TYPE carry = 0;
    const EK_BIGINT_TYPE high_bit = (((EK_BIGINT_TYPE) 1) << (8*sizeof(EK_BIGINT_TYPE) - 1));

    for (int i = 0; i <= a->last; i++) {
        EK_BIGINT_TYPE v = a->data[i];
        a->data[i] = (a->data[i] << 1) + carry;
        carry = (v & high_bit) ? 1 : 0;
    }

    if (carry) {
        assert(a->last + 1 < a->len);
        a->data[++a->last] = 1;
    }

    ek_bigint_validate(a);
}

// a = (a << 1) % mod
// assumes 'a' is already reduced modulo 'mod'
void ek_bigint_mul_2_mod_inplace(ek_bigint_t *a, const ek_bigint_t *mod)
{
    ek_bigint_validate(a);
    ek_bigint_validate(mod);

    ek_bigint_ensure_nwords(a, a->last + 2);

    EK_BIGINT_TYPE carry = 0;
    const EK_BIGINT_TYPE high_bit = (((EK_BIGINT_TYPE) 1) << (8*sizeof(EK_BIGINT_TYPE) - 1));

    for (int i = 0; i <= a->last; i++) {
        EK_BIGINT_TYPE v = a->data[i];
        a->data[i] = (a->data[i] << 1) + carry;
        carry = (v & high_bit) ? 1 : 0;
    }

    if (carry)
        a->data[++a->last] = 1;

    ek_bigint_validate(a);

    // should only execute once.
    if (!ek_bigint_less_than(a, mod))
        ek_bigint_sub_inplace(a, mod);

    assert(ek_bigint_less_than(a, mod));
}

// a = a / 2
void ek_bigint_div_2_inplace(ek_bigint_t *a)
{
    ek_bigint_validate(a);

    EK_BIGINT_TYPE carry = 0;
    const EK_BIGINT_TYPE high_bit = (((EK_BIGINT_TYPE) 1) << (8*sizeof(EK_BIGINT_TYPE) - 1));

    for (int i = a->last; i >= 0; i--) {
        EK_BIGINT_TYPE v = a->data[i];
        a->data[i] = (a->data[i] >> 1) + carry;
        carry = (v & 1) ? high_bit : 0;
    }

    while (a->last > 0 && a->data[a->last] == 0)
        a->last--;

    ek_bigint_validate(a);
}

// a = a - b. b is less than or equal to a.
void ek_bigint_sub_inplace(ek_bigint_t *a, const ek_bigint_t *b)
{
    assert(ek_bigint_less_equal(b, a));
    ek_bigint_validate(a);
    ek_bigint_validate(b);

    int borrow = 0;

    assert(a->last < a->len);

    for (int i = 0; i <= b->last; i++) {

        EK_BIGINT_TYPE acc1 = a->data[i];
        EK_BIGINT_TYPE acc2 = acc1 - b->data[i];
        EK_BIGINT_TYPE acc3 = acc2 - borrow;

        // we underflowed if, at any point, we SET the high bit.
        if (acc3 > acc1 || acc3 > acc2 || acc2 > acc1)
            borrow = 1;
        else
            borrow = 0;

        a->data[i] = acc3;
    }

    for (int i = b->last + 1; borrow; i++) {
        if (a->data[i] != 0)
            borrow = 0;

        a->data[i]--;
    }

    while (a->last > 0 && a->data[a->last] == 0)
        a->last--;

    ek_bigint_validate(a);
}

// a = a + b
void ek_bigint_add_inplace(ek_bigint_t *a, const ek_bigint_t *b)
{
    ek_bigint_validate(a);
    ek_bigint_validate(b);

    ek_bigint_ensure_nwords(a, imax(a->last, b->last) + 2);

    // worst-case length of a
    assert(a->last < a->len);

    int carry = 0;
    for (int i = 0; i <= b->last; i++) {

        EK_BIGINT_TYPE acc1 = a->data[i];
        EK_BIGINT_TYPE acc2 = acc1 + b->data[i];
        EK_BIGINT_TYPE acc3 = acc2 + carry;

        if (acc3 < acc1 || acc3 < acc2 || acc2 < acc1)
            carry = 1;
        else
            carry = 0;

        a->data[i] = acc3;
    }

    for (int i = b->last + 1; carry; i++) {
        a->data[i]++;
        if (a->data[i] != 0)
            break;
    }

    // look for last non-zero value by starting at the last entry that could
    // possibly be non-zero.
    a->last = imax(a->last, b->last) + 1;
    while (a->last > 0 && a->data[a->last] == 0)
        a->last--;

    ek_bigint_validate(a);
}

// a = (a + b) % m
// assumes a and b are already reduced moduluo 'mod'
void ek_bigint_add_mod_inplace(ek_bigint_t *a, const ek_bigint_t *b, const ek_bigint_t *mod)
{
    assert(ek_bigint_less_than(a, mod));
    assert(ek_bigint_less_than(b, mod));

    ek_bigint_add_inplace(a, b);

    if (!ek_bigint_less_than(a, mod))
        ek_bigint_sub_inplace(a, mod);

    assert(ek_bigint_less_than(a, mod));
}

int ek_bigint_is_zero(const ek_bigint_t *a)
{
    return (a->last == 0 && a->data[0] == 0);
}

// r = v mod m
ek_bigint_t *ek_bigint_mod(const ek_bigint_t *v, const ek_bigint_t *mod)
{
    assert(!ek_bigint_is_zero(mod));

    ek_bigint_validate(v);
    ek_bigint_validate(mod);
    ek_bigint_t *m2 = ek_bigint_copy(mod);
    ek_bigint_ensure_nwords(m2, v->last + 2);

    ek_bigint_t *remainder = ek_bigint_copy(v);

    // repeatedly multiply 'm2' by 2 until it is larger or equal to v.
    // how many positions have we shifted?
    int cnt = 0;

    // first shift by whole words
    if (1) {
        // how many words to shift?
        int words = v->last - m2->last;
        if (m2->data[m2->last] < v->data[v->last])
            words++;

        if (words > 0) {

            for (int i = m2->last; i >= 0; i--)
                m2->data[i + words] = m2->data[i];

            for (int i = 0; i < words; i++)
                m2->data[i] = 0;

            m2->last += words;
            cnt += words * sizeof(EK_BIGINT_TYPE) * 8;
        }
        ek_bigint_validate(m2);
    }

    // now, bit by bit
    while (ek_bigint_less_than(m2, remainder)) {
        ek_bigint_mul_2_inplace(m2);
        cnt ++;
    }

    for (int i = 0; i <= cnt; i++) {
        if (ek_bigint_less_equal(m2, remainder))
            ek_bigint_sub_inplace(remainder, m2);
        ek_bigint_div_2_inplace(m2);
    }

    ek_bigint_destroy(m2);

    ek_bigint_validate(remainder);
    return remainder;
}

// put the smaller number in b for better performance.
ek_bigint_t *ek_bigint_mul(const ek_bigint_t *a, const ek_bigint_t *b)
{
    ek_bigint_validate(a);
    ek_bigint_validate(b);

    ek_bigint_t *r = ek_bigint_create_nwords(a->last + b->last + 3);
    ek_bigint_t *a2 = ek_bigint_copy(a);

    int bits_per_word = sizeof(EK_BIGINT_TYPE) * 8;
    for (int bitidx = 0; bitidx < (b->last + 1) * bits_per_word; bitidx++) {
        int word_idx = bitidx / bits_per_word;
        int word_bit = bitidx & (bits_per_word - 1);

        int bit = (b->data[word_idx] >> word_bit) & 0x01;
        if (bit)
            ek_bigint_add_inplace(r, a2);

        ek_bigint_mul_2_inplace(a2);
    }

    ek_bigint_destroy(a2);

    return r;
}

// doesn't update a->last.
static inline void ek_bigint_add_word_nolast(ek_bigint_t *a, int idx, EK_BIGINT_TYPE v)
{
    EK_BIGINT_TYPE oldv = a->data[idx];

    a->data[idx] += v;
    if (a->data[idx] < oldv) {
        int carryidx = 0;
        do {
            carryidx++;
            assert(idx + carryidx < a->len);
            a->data[idx + carryidx]++;
        } while (a->data[idx + carryidx] == 0);
    }
}

struct ek_bigint_mul_lookup *ek_bigint_mul_lookup_create(const ek_bigint_t *mod, int tablelen)
{
    ek_bigint_t **table = calloc(tablelen, sizeof(ek_bigint_t*));

    table[0] = ek_bigint_create_int(1);

    for (int i = 1; i < tablelen; i++) {
        table[i] = ek_bigint_copy(table[i-1]);

        ek_bigint_ensure_nwords(table[i], table[i]->len + 1);

        // multiply by 2^word size (by shifting a whole word.)
        for (int j = table[i]->len - 1; j >= 1; j--) {
            table[i]->data[j] = table[i]->data[j-1];
        }

        table[i]->data[0] = 0;
        ek_bigint_fix_last(table[i]);

        ek_bigint_t *tmp = table[i];
        table[i] = ek_bigint_mod(table[i], mod);
        ek_bigint_destroy(tmp);
    }

    struct ek_bigint_mul_lookup *lookup = calloc(1, sizeof(struct ek_bigint_mul_lookup));
    lookup->tablelen = tablelen;
    lookup->table = table;

    return lookup;
}

void ek_bigint_mul_lookup_destroy(struct ek_bigint_mul_lookup *lookup)
{
    if (!lookup)
        return;

    for (int i = 0; i < lookup->tablelen; i++)
        ek_bigint_destroy(lookup->table[i]);
    free(lookup->table);
    free(lookup);
}

// XXX Unsure how to avoid (false) compiler warnings for small
// settings of EK_BIGINT_TYPE.
static inline void word_multiply(EK_BIGINT_TYPE a, EK_BIGINT_TYPE b, EK_BIGINT_TYPE *_hi, EK_BIGINT_TYPE *_lo)
{
    if (sizeof(EK_BIGINT_TYPE) == 1) {
        uint16_t ab = a;
        ab *= b;
        *_lo = ab;
        *_hi = ab >> 8;
        return;
    }

    if (sizeof(EK_BIGINT_TYPE) == 2) {
        uint32_t ab = a;
        ab *= b;
        *_lo = ab;
        *_hi = ab >> 16;
        return;
    }

    if (sizeof(EK_BIGINT_TYPE) == 4) {
        #ifdef ARCH_32BIT
        __int64_t ab = a;
        #else
        uint64_t ab = a;
        #endif
        ab *= b;
        *_lo = ab;
        *_hi = ab >> 32;
        return;
    }


#ifndef ARCH_32BIT

    if (sizeof(EK_BIGINT_TYPE) == 8) {
        // wx * yz = 00xz      (each letter represents 32 bits)
        //           0wz0
        //           0xy0
        //           wy00
/*
        uint64_t w = a >> 32, x = a & 0xffffffff;
        uint64_t y = b >> 32, z = b & 0xffffffff;

        uint64_t xz = x*z, wz = w*z, xy = x*y, wy = w*y;

        uint64_t lo = xz;
        uint64_t hi = wy + (wz >> 32) + (xy >> 32);

        uint64_t tmp;
        tmp = lo;
        lo += (wz << 32);
        if (lo < tmp)
            hi++;

        tmp = lo;
        lo += (xy << 32);
        if (lo < tmp)
            hi++;

        *_lo = lo;
        *_hi = hi;
        */

        __int128_t v = a;
        v *= b;
        *_lo = v;
        *_hi = v >> 64;
        return;
    }

#endif

    assert(0);
}

// if 'weak' is set, the resulting answer will be only partially
// reduced modulo 'mod'.  It will be up to a factor of
// nbits*2^wordbits larger than the modulus. (This is useful because
// reducing it all the way modulo 'mod' is much more expensive, and
// for modular exponentiation, it doesn't hurt us (much) if our
// intermediate terms are not fully reduced modulo 'mod'. Basically,
// you need to allow two extra words of storage.
ek_bigint_t *ek_bigint_mul_mod_lookup(const ek_bigint_t *a, const ek_bigint_t *b,
                                      const struct ek_bigint_mul_lookup *lookup, const ek_bigint_t *mod, int weak)
{
    ek_bigint_validate(a);
    ek_bigint_validate(b);
    ek_bigint_validate(mod);

    // idea: Suppose we have 32 bit words. Precompute 2^N mod M for 0,
    // 32, 64, etc. (At each step, we take the residue from the
    // previous step, multiply by 2^32, then reduce again.  This
    // requires O(32) compares and subtractions
    //
    // Then do a full multiply. For each word in the result, we
    // collect the residues and scale by the value of each word, then
    // reduce again.
    //
    // This should be relatively fast because we only need to do 32 +
    // log(N) reductions. (?)
    ek_bigint_t **table = lookup->table;

    // do the multiplication itself, product = a * b. (Which is not reduced modulo anything!)
    ek_bigint_t *product = ek_bigint_create_nwords(a->last + b->last + 3);

    for (int ai = 0; ai <= a->last; ai++) {
        for (int bi = 0; bi <= b->last; bi++) {

            EK_BIGINT_TYPE lo, hi;
            word_multiply(a->data[ai], b->data[bi], &hi, &lo);

            ek_bigint_add_word_nolast(product, ai + bi, lo);
            ek_bigint_add_word_nolast(product, ai + bi + 1, hi);
        }
    }
    product->last = product->len - 1;
    while (product->last > 0 && product->data[product->last] == 0)
        product->last--;

    // do the modular reduction using the table
    ek_bigint_t *r = ek_bigint_create_nwords(mod->last + 3);

    assert(product->last < lookup->tablelen);

    for (int wordidx = 0; wordidx <= product->last; wordidx++) {

        // r += table[wordidx] * product->data[wordidx]
        for (int i = 0; i <= table[wordidx]->last; i++) {
            EK_BIGINT_TYPE lo, hi;
            word_multiply(product->data[wordidx], table[wordidx]->data[i], &hi, &lo);

            ek_bigint_add_word_nolast(r, i, lo);
            ek_bigint_add_word_nolast(r, i + 1, hi);
        }
    }
    ek_bigint_fix_last(r);

    ek_bigint_destroy(product);

    if (!weak) {
        ek_bigint_t *tmp = r;
        r = ek_bigint_mod(tmp, mod);
        ek_bigint_destroy(tmp);
    }

    return r;
}

// assumes that 'a' and 'b' are already reduced modulo 'modulus'
ek_bigint_t *ek_bigint_mul_mod(const ek_bigint_t *a, const ek_bigint_t *b, const ek_bigint_t *mod)
{
    ek_bigint_validate(a);
    ek_bigint_validate(b);
    ek_bigint_validate(mod);

    assert(ek_bigint_less_than(a, mod));
    assert(ek_bigint_less_than(b, mod));

    ek_bigint_t *r = ek_bigint_create_nwords(mod->last + 2);
    ek_bigint_t *a2 = ek_bigint_copy(a);

    int bits_per_word = sizeof(EK_BIGINT_TYPE) * 8;
    for (int bitidx = 0; bitidx < (b->last + 1) * bits_per_word; bitidx++) {
        int word_idx = bitidx / bits_per_word;
        int word_bit = bitidx & (bits_per_word - 1);

        int bit = (b->data[word_idx] >> word_bit) & 0x01;

        if (bit)
            ek_bigint_add_mod_inplace(r, a2, mod);

        ek_bigint_mul_2_mod_inplace(a2, mod);
    }

    ek_bigint_destroy(a2);

    ek_bigint_fix_last(r);
    return r;
}

// r = a^p mod m
ek_bigint_t *ek_bigint_pow_mod(const ek_bigint_t *a, const ek_bigint_t *p, const ek_bigint_t *mod)
{
    ek_bigint_validate(a);
    ek_bigint_validate(p);
    ek_bigint_validate(mod);

    if (!ek_bigint_less_than(a, mod)) {
//        printf("warning: a > mod\n");
        ek_bigint_t *am = ek_bigint_mod(a, mod);
        ek_bigint_t *res = ek_bigint_pow_mod(am, p, mod);
        ek_bigint_destroy(am);
        return res;
    }

    assert(ek_bigint_less_than(a, mod));

    struct ek_bigint_mul_lookup *lookup = ek_bigint_mul_lookup_create(mod, (mod->last + 3) * 2 + 1);

    ek_bigint_t *r = ek_bigint_create_int(1); // r = a^0 mod m = 1
    ek_bigint_t *apow = ek_bigint_copy(a);

    int bits_per_word = sizeof(EK_BIGINT_TYPE) * 8;

    for (int bitidx = 0; bitidx < (p->last + 1) * bits_per_word; bitidx++) {
        int word_idx = bitidx / bits_per_word;
        int word_bit = bitidx & (bits_per_word - 1);

        int bit = (p->data[word_idx] >> word_bit) & 0x01;

        if (bit) {
            ek_bigint_t *tmp = ek_bigint_mul_mod_lookup(r, apow, lookup, mod, 1);
            ek_bigint_destroy(r);
            r = tmp;
        }

        if (1) {
            ek_bigint_t *tmp = ek_bigint_mul_mod_lookup(apow, apow, lookup, mod, 1);
            ek_bigint_destroy(apow);
            apow = tmp;
        }
    }

    if (1) {
        ek_bigint_t *tmp = ek_bigint_mod(r, mod);
        ek_bigint_destroy(r);
        r = tmp;
    }

    ek_bigint_destroy(apow);
    ek_bigint_mul_lookup_destroy(lookup);
    return r;
}

// how many bits (minimum) would be required to represent this number?
int ek_bigint_nbits(const ek_bigint_t *a)
{
    int nbits = 0;
    for (int idx = 0; idx < a->len; idx++) {
        for (int bitidx = 0; bitidx < sizeof(EK_BIGINT_TYPE)*8; bitidx++) {
            if ((a->data[idx] >> bitidx) & 1)
                nbits = idx * sizeof(EK_BIGINT_TYPE)*8 + bitidx;
        }
    }
    // if the MSB is in the zeroth bit, it requires one bit.
    return nbits + 1;
}

static int int2hexchar(int v)
{
    if (v < 10)
        return v + '0';
    return v + 'A' - 10;
}

char *ek_bigint_to_string_hex(ek_bigint_t *a)
{
    int nbytes = sizeof(EK_BIGINT_TYPE)*a->len;
    char *s = calloc(1, nbytes * 2 + 1);
    int spos = 0;

    int nz = 0;

    for (int i = nbytes - 1; i >= 0; i--) {
        int v = ek_bigint_get_byte(a, i);
        nz |= v;
        if (nz) {
            s[spos++] = int2hexchar(v >> 4);
            s[spos++] = int2hexchar(v & 15);
        }
    }

    if (!nz)
        s[spos++] = '0';

    s[spos] = 0;

    return s;
}
