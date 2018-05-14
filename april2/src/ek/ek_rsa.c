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
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include "ek_bigint.h"
#include "ek_hash.h"

// k: size of the modulus (in bytes). E.g., 1024 bit RSA => k=128
// padding: up to 'k-3' bytes of NON-ZERO random padding. (Simplicity: just provide k bytes)
// in: the data to be encrypted, between 0 and k-11 bytes.
// inlen: the length of 'in'.
// out: k bytes of output.
//
// returns non-zero if something goes wrong.
int ek_rsa_pkcs_15_encrypt(const ek_bigint_t *modulus, const ek_bigint_t *pubexp,
                           const uint8_t *padding, const uint8_t *in, int inlen, uint8_t *out, int *out_len)
{
    // PKCS defines k as the number of octets "touched" (not "filled")
    // by the modulus.  This is okay because the first byte of padding
    // is always 0x00.
    int k = (ek_bigint_nbits(modulus) + 7) / 8;

    // construct the (padded) message by concatenating:
    // 0x00 || 0x02 || PADDING || 0x00 || M
    //
    // Where enough padding bytes are added so that the total message
    // is 'k' octets long.

    ek_bigint_t *msg = ek_bigint_create_int(0);
    ek_bigint_set_byte(msg, k - 1, 0x00);
    ek_bigint_set_byte(msg, k - 2, 0x02);
    int padlen = k - 3 - inlen;
    if (padlen < 8) // "message too long"
        return -1;

    // padding
    for (int i = 0; i < padlen; i++)
        ek_bigint_set_byte(msg, k - 3 - i, padding[i]);

    ek_bigint_set_byte(msg, k - 3 - padlen, 0x00);

    for (int i = 0; i < inlen; i++)
        ek_bigint_set_byte(msg, k - 4 - padlen - i, in[i]);

    ek_bigint_t *enc = ek_bigint_pow_mod(msg, pubexp, modulus);

    for (int i = 0; i < k; i++)
        out[i] = ek_bigint_get_byte(enc, k - 1 - i);

    *out_len = k;
    return 0;
}

static int pkcs_decrypt_unpack(int k, const ek_bigint_t *dec_pad, uint8_t *out, int *outlen)
{
    if (ek_bigint_get_byte(dec_pad, k - 1) != 0x00)
        return -1;

    if (ek_bigint_get_byte(dec_pad, k - 2) != 0x02)
        return -2;

    // find the zero after the padding string
    int offset = k - 3;
    while (offset >= 0 && ek_bigint_get_byte(dec_pad, offset) != 0)
        offset--;
    offset--;

    // no zero?
    if (offset < 0)
        return -3;

    *outlen = 0;
    while (offset >= 0) {
        out[*outlen] = ek_bigint_get_byte(dec_pad, offset);
        offset--;
        (*outlen)++;
    }

    return 0;
}

// k: size of modulus
// in: k bytes of encrypted input
// out: a buffer of size 'k', which will be written to.
// outlen: the number of valid bytes in 'out'
// returns non-zero if something goes wrong
int ek_rsa_pkcs_15_decrypt_privexp(const ek_bigint_t *modulus, const ek_bigint_t *privexp,
                                   const uint8_t *_in, uint8_t *out, int *outlen)
{
    int k = (ek_bigint_nbits(modulus) + 7) / 8;

    ek_bigint_t *in = ek_bigint_create_int(0);

    // MSB is the first byte
    for (int i = 0; i < k; i++)
        ek_bigint_set_byte(in, i, _in[k - 1- i]);

    ek_bigint_t *dec = ek_bigint_pow_mod(in, privexp, modulus);

    int ret = pkcs_decrypt_unpack(k, dec, out, outlen);

    ek_bigint_destroy(dec);
    ek_bigint_destroy(in);

    return ret;
}

int ek_rsa_pkcs_15_decrypt_crt(const ek_bigint_t *modulus,
                               const ek_bigint_t *p, const ek_bigint_t *q,
                               const ek_bigint_t *dP, const ek_bigint_t *dQ,
                               const ek_bigint_t *qInv,
                               const uint8_t *_in, uint8_t *out, int *outlen)
{
    int k = (ek_bigint_nbits(modulus) + 7) / 8;

    ek_bigint_t *in = ek_bigint_create_int(0);

    // MSB is the first byte
    for (int i = 0; i < k; i++)
        ek_bigint_set_byte(in, i, _in[k - 1- i]);

    // m1 = c^dP mod p
    ek_bigint_t *m1 = ek_bigint_pow_mod(in, dP, p);

    // m2 = c^dQ mod q
    ek_bigint_t *m2 = ek_bigint_pow_mod(in, dQ, q);

    // h = (m1 - m2) * qInv mod p
    ek_bigint_t *tmp = ek_bigint_copy(m1);
    while (ek_bigint_less_than(tmp, m2))
        ek_bigint_add_inplace(tmp, p);

    ek_bigint_sub_inplace(tmp, m2);

    ek_bigint_t *h = ek_bigint_mul_mod(tmp, qInv, p);

    // m = m2 + q*h (mod pq)
    ek_bigint_t *m = ek_bigint_mul_mod(q, h, modulus);
    ek_bigint_add_inplace(m, m2);

    int ret = pkcs_decrypt_unpack(k, m, out, outlen);

    ek_bigint_destroy(in);
    ek_bigint_destroy(m1);
    ek_bigint_destroy(m2);
    ek_bigint_destroy(tmp);
    ek_bigint_destroy(h);
    ek_bigint_destroy(m);

    return ret;
}

// returns non-zero if something goes wrong. In should be the DER formatted data.
int ek_rsa_ssa_pkcs_15_encrypt_der(const ek_bigint_t *modulus, const ek_bigint_t *privexp,
                                  const uint8_t *in, int inlen, uint8_t *out, int *out_len)
{
    // PKCS defines k as the number of octets "touched" (not "filled")
    // by the modulus.  This is okay because the first byte of padding
    // is always 0x00.
    int k = (ek_bigint_nbits(modulus) + 7) / 8;

    // construct the (padded) message by concatenating:
    // 0x00 || 0x01 || PADDING || 0x00 || M
    //
    // Where enough padding bytes are added so that the total message
    // is 'k' octets long.

    ek_bigint_t *msg = ek_bigint_create_int(0);
    ek_bigint_set_byte(msg, k - 1, 0x00);
    ek_bigint_set_byte(msg, k - 2, 0x01);
    int padlen = k - 3 - inlen;
    if (padlen < 8) // "message too long"
        return -1;

    // padding
    for (int i = 0; i < padlen; i++)
        ek_bigint_set_byte(msg, k - 3 - i, 0xff);

    ek_bigint_set_byte(msg, k - 3 - padlen, 0x00);

    for (int i = 0; i < inlen; i++)
        ek_bigint_set_byte(msg, k - 4 - padlen - i, in[i]);

    ek_bigint_t *enc = ek_bigint_pow_mod(msg, privexp, modulus);

    for (int i = 0; i < k; i++)
        out[i] = ek_bigint_get_byte(enc, k - 1 - i);

    *out_len = k;

    return 0;
}


int ek_rsa_ssa_pkcs_15_sign_sha1(const ek_bigint_t *modulus, const ek_bigint_t *privexp,
                                 const uint8_t *in, int inlen, uint8_t *out, int *out_len)
{
    ek_hash_algorithm_t _alg, *alg = &_alg;
    if (ek_hash_algorithm_get("SHA1", &_alg)) {
        printf("crap, no sha1\n");
    }

    uint8_t digest[alg->digest_size];
    ek_hash_state_t state;
    alg->init(&state);
    alg->update(&state, in, inlen);
    alg->final(&state, digest);

    // only correct for sha-1. see rfc3447 sec 9.2
    uint8_t derhdr[] = { 0x30, 0x21, 0x30, 0x09, 0x06, 0x05, 0x2b, 0x0e,
                         0x03, 0x02, 0x1a, 0x05, 0x00, 0x04, 0x14};

    int derlen = alg->digest_size + sizeof(derhdr);
    uint8_t *der = malloc(derlen);
    memcpy(der, derhdr, sizeof(derhdr));
    memcpy(&der[sizeof(derhdr)], digest, alg->digest_size);

    uint8_t sig[4096];
    int siglen;
    return ek_rsa_ssa_pkcs_15_encrypt_der(modulus, privexp,
                                          der, derlen, out, out_len);
}

int ek_rsa_ssa_pkcs_15_decrypt_der(const ek_bigint_t *modulus, const ek_bigint_t *pubexp,
                                   const uint8_t *_in, uint8_t *out, int *outlen)
{
    int k = (ek_bigint_nbits(modulus) + 7) / 8;

    ek_bigint_t *in = ek_bigint_create_int(0);

    // MSB is the first byte
    for (int i = 0; i < k; i++)
        ek_bigint_set_byte(in, i, _in[k - 1- i]);

    ek_bigint_t *dec = ek_bigint_pow_mod(in, pubexp, modulus);

    ek_bigint_destroy(dec);
    ek_bigint_destroy(in);

//    return ret;
    return 0;
}
