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

#ifndef _EK_H
#define _EK_H

#include <stdint.h>

// fills buffer with secure random data
void ek_random_buffer(void *buf, int len);

// allocates a buffer of size len and fills it with secure random bytes
void *ek_random_buffer_create(int len);

uint8_t ek_random_u8();

void ek_print_buffer_indent(const void *_data, int datalen, int nindent, const char *indent);
void ek_print_buffer(const void *_data, int datalen);
int ek_ascii_to_bytes(const char *s, uint8_t **pdata, int *plen);

#include "ek_asn.h"
#include "ek_bigint.h"

typedef struct ek_identity ek_identity_t;
struct ek_identity {
    ek_asn_result_t *cert;
    ek_asn_result_t *privkey;

    struct {
        ek_bigint_t *modulus;
        ek_bigint_t *privexp, pubexp;
        ek_bigint_t *prime1, *prime2, *exponent1, *exponent2, *coefficient;
    } rsa;
};


#include "ek_hash.h"
#include "ek_rsa.h"
#include "ek_block_cipher.h"
#include "ek_prf.h"
#include "ek_tls.h"

ek_identity_t *ek_identity_create(const char *cert_der_path, const char *privkey_der_path);

#endif
