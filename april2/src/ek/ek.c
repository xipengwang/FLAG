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
#include <stdio.h>

#include "common/io_util.h"

#include "ek.h"

void ek_random_buffer(void *buf, int len)
{
    FILE *f = fopen("/dev/urandom", "r");

    if (fread(buf, 1, len,  f) != len)
        exit(-1); // fatal!

    fclose(f);
}

void *ek_random_buffer_create(int len)
{
    char *buf = malloc(len);
    ek_random_buffer(buf, len);
    return buf;
}

uint8_t ek_random_u8()
{
    uint8_t v;
    ek_random_buffer(&v, 1);
    return v;
}

void ek_print_buffer_indent(const void *_data, int datalen, int nindent, const char *indent)
{
    const uint8_t *data = _data;

    if (datalen == 0) {
        for (int i = 0; i < nindent; i++)
            printf("%s", indent);

        printf("0000: <none>\n");
    }

    for (int i = 0; i < datalen; i++) {
        if ((i % 16) == 0) {
            for (int i = 0; i < nindent; i++)
                printf("%s", indent);

            printf("%04x: ", i);
        }

        printf("%02x ", data[i]);
        if ((i % 16) == 15)
            printf("\n");
    }

    printf("\n");
}

void ek_print_buffer(const void *_data, int datalen)
{
    ek_print_buffer_indent(_data, datalen, 0, "");
}

static int chartohex(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';

    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;

    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;

    return -1;
}

int ek_ascii_to_bytes(const char *s, uint8_t **pdata, int *plen)
{
    uint8_t *data = NULL;
    int datalen = 0;

    int pos = 0;

    while (s[pos] != 0) {

        int c0 = -1, c1 = -1;
        while (s[pos] != 0 && ((c0 = chartohex(s[pos++])) < 0));

        while (s[pos] != 0 && ((c1 = chartohex(s[pos++])) < 0));

        if (c0 < 0 || c1 < 0)
            break;

        datalen++;
        data = realloc(data, sizeof(uint8_t)*datalen);
        data[datalen - 1] = (c0 << 4) + c1;
    }

    *pdata = data;
    *plen = datalen;
    return 0;
}

ek_identity_t *ek_identity_create(const char *cacert_der_path, const char *privkey_der_path)
{
    ek_identity_t *ident = calloc(1, sizeof(ek_identity_t));

    if (1) {
        char *buf;
        long buf_len;

        if (ioutils_read_file(cacert_der_path, &buf, &buf_len, -1)) {
            goto fail;
        }

        int error;
        ident->cert = ek_asn_result_create_from_der(buf, buf_len, ek_asn_type_create_x509_cacert(), &error);
        if (!ident->cert) {
            printf("couldn't parse certificate (%d)\n", error);
            free(buf);
            goto fail;
        }
    }

    if (1) {
        uint8_t *buf;
        long buf_len;

        if (ioutils_read_file(privkey_der_path, (char**) &buf, &buf_len, -1)) {
            goto fail;
        }

        int error;
        ident->privkey = ek_asn_result_create_from_der(buf, buf_len, ek_asn_type_create_x509_privkey(), &error);
        if (!ident->privkey) {
            printf("couldn't parse privkey (%d)\n", error);
            free(buf);
            goto fail;
        }

        struct pair {
            const char *name;
            ek_bigint_t **dest;
        };

        struct pair *pairs = (struct pair[]) { { .name = "privkey.modulus", .dest = &ident->rsa.modulus },
                                               { .name = "privkey.privateExponent", .dest = &ident->rsa.privexp },
                                               { .name = "privkey.prime1", .dest = &ident->rsa.prime1 },
                                               { .name = "privkey.prime2", .dest = &ident->rsa.prime2 },
                                               { .name = "privkey.exponent1", .dest = &ident->rsa.exponent1 },
                                               { .name = "privkey.exponent2", .dest = &ident->rsa.exponent2 },
                                               { .name = "privkey.coefficient", .dest = &ident->rsa.coefficient },
                                               { .name = NULL } };

        for (int i = 0; pairs[i].name != NULL; i++) {
            struct ek_asn_element *el;
            if (ek_asn_result_find(ident->privkey, pairs[i].name, &el)) {
                printf("Unable to retrieve keys from privkey file: %s\n", pairs[i].name);
                goto fail;
            }

            *pairs[i].dest = ek_bigint_create_from_bytes(el->payload, el->payload_len);
        }
    }

    return ident;

  fail:
    ek_asn_result_destroy(ident->cert);
    ek_asn_result_destroy(ident->privkey);
    free(ident);
    return NULL;
}
