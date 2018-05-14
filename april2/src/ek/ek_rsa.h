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

#ifndef _EK_RSA_H
#define _EK_RSA_H

#include "ek_bigint.h"

// k: size of the modulus (in bytes). E.g., 1024 bit RSA => k=128
// padding: up to 'k-3' bytes of NON-ZERO random padding. (Simplicity: just provide k bytes)
// in: the data to be encrypted, between 0 and k-11 bytes.
// inlen: the length of 'in'.
// out: k bytes of output.
//
// returns non-zero if something goes wrong.
int ek_rsa_pkcs_15_encrypt(const ek_bigint_t *modulus, const ek_bigint_t *pubexp,
                           const uint8_t *padding, const uint8_t *in, int inlen, uint8_t *out, int *out_len);


// k: size of modulus
// in: k bytes of encrypted input
// out: a buffer of size 'k', which will be written to.
// outlen: the number of valid bytes in 'out'
// returns non-zero if something goes wrong
int ek_rsa_pkcs_15_decrypt_privexp(const ek_bigint_t *modulus, const ek_bigint_t *privexp,
                                   const uint8_t *in, uint8_t *out, int *outlen);

// about 2.5x faster than the privexp version
int ek_rsa_pkcs_15_decrypt_crt(const ek_bigint_t *modulus,
                               const ek_bigint_t *p, const ek_bigint_t *q,
                               const ek_bigint_t *dP, const ek_bigint_t *dQ,
                               const ek_bigint_t *qInv,
                               const uint8_t *in, uint8_t *out, int *outlen);


int ek_rsa_ssa_pkcs_15_encrypt_der(const ek_bigint_t *modulus, const ek_bigint_t *privexp,
                                  const uint8_t *in, int inlen, uint8_t *out, int *out_len);


// XXX unfinished
int ek_rsa_ssa_pkcs_15_decrypt_der(const ek_bigint_t *modulus, const ek_bigint_t *pubexp,
                                   const uint8_t *_in, uint8_t *out, int *outlen);

int ek_rsa_ssa_pkcs_15_sign_sha1(const ek_bigint_t *modulus, const ek_bigint_t *privexp,
                                 const uint8_t *in, int inlen, uint8_t *out, int *out_len);

#endif
