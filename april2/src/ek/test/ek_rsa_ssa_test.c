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
#include <stdint.h>
#include <sys/time.h>

#include "../ek.h"

#include "../../common/string_util.h"

int64_t utime_now() // blacklist-ignore
{
    struct timeval tv;
    gettimeofday (&tv, NULL); // blacklist-ignore
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}


// read one or more lines containing something other than spaces,
// concatenating them together.
char *read_ascii_block(FILE *f)
{
    char *acc = strdup("");

    char _buf[1024];
    while (NULL != fgets(_buf, sizeof(_buf), f)) {
        char *buf = str_trim(_buf);
        if (strlen(buf) == 0)
            break;

        char *tmp = acc;
        acc = sprintf_alloc("%s %s", acc, buf);
        free(tmp);
    }

    return acc;
}

ek_bigint_t *make_bigint(FILE *f)
{
    char *s = read_ascii_block(f);
    ek_bigint_t *ret = ek_bigint_create_from_ascii(s);
    free(s);
    return ret;
}

int main(int argc, char *argv[])
{
    char *path = strdup(argv[0]);
    char *slash = strrchr(path, '/');
    if (slash) {
        *slash = 0;
    }

    char vectors_path[1024];
    sprintf(vectors_path, "%s/%s", path, "pkcs1v15sign-vectors.txt");

    FILE *f = fopen(vectors_path, "r");

    ek_bigint_t *modulus = NULL;
    ek_bigint_t *pubexp = NULL;
    ek_bigint_t *privexp = NULL;
    ek_bigint_t *prime1 = NULL;
    ek_bigint_t *prime2 = NULL;
    ek_bigint_t *primeexp1 = NULL;
    ek_bigint_t *primeexp2 = NULL;
    ek_bigint_t *coeff = NULL;

    uint8_t *message = NULL;
    int messagelen;

    uint8_t *seed = NULL;
    int seedlen;

    uint8_t *encryption = NULL;
    int encryptionlen;

    char *example_name = NULL;

    int64_t enc_usecs = 0;
    int64_t dec_privexp_usecs = 0;
    int64_t dec_crt_usecs = 0;

    int nops = 0;

    char _buf[1024];
    while (NULL != fgets(_buf, sizeof(_buf), f)) {
        char *buf = str_trim(_buf);

        if (strlen(buf) == 0)
            goto cleanup;

        // These are essentially comment lines... skip them
        if (str_starts_with(buf, "# PKCS")) {
            free(example_name);
            example_name = strdup(buf);
            goto cleanup;
        }

        if (str_starts_with(buf, "# --"))
            goto cleanup;

        if (str_starts_with(buf, "# =="))
            goto cleanup;

        if (str_starts_with(buf, "# Example"))
            goto cleanup;

        if (str_starts_with(buf, "# Private key"))
            goto cleanup;

        ////////////////////////////////////////////////
        // these lines include some data which we will grab!
        if (str_starts_with(buf, "# Modulus")) {
            ek_bigint_destroy(modulus);
            modulus = make_bigint(f);
            goto cleanup;
        }

        if (str_starts_with(buf, "# Public")) {
            ek_bigint_destroy(pubexp);
            pubexp = make_bigint(f);
            goto cleanup;
        }

        if (str_starts_with(buf, "# Exponent")) {
            ek_bigint_destroy(privexp);
            privexp = make_bigint(f);
            goto cleanup;
        }

        if (str_starts_with(buf, "# Prime 1")) {
            ek_bigint_destroy(prime1);
            prime1 = make_bigint(f);
            goto cleanup;
        }

        if (str_starts_with(buf, "# Prime 2")) {
            ek_bigint_destroy(prime2);
            prime2 = make_bigint(f);
            goto cleanup;
        }

        if (str_starts_with(buf, "# Prime exponent 1")) {
            ek_bigint_destroy(primeexp1);
            primeexp1 = make_bigint(f);
            goto cleanup;
        }

        if (str_starts_with(buf, "# Prime exponent 2")) {
            ek_bigint_destroy(primeexp2);
            primeexp2 = make_bigint(f);
            goto cleanup;
        }

        if (str_starts_with(buf, "# Coefficient")) {
            ek_bigint_destroy(coeff);
            coeff = make_bigint(f);
            goto cleanup;
        }

        if (str_starts_with(buf, "# Message")) {
            char *s = read_ascii_block(f);
            free(message);
            ek_ascii_to_bytes(s, &message, &messagelen);
            free(s);

            goto cleanup;
        }

        if (str_starts_with(buf, "# Seed")) {
            char *s = read_ascii_block(f);
            free(seed);
            ek_ascii_to_bytes(s, &seed, &seedlen);
            free(s);

            goto cleanup;
        }

        if (str_starts_with(buf, "# Signature")) {
            char *s = read_ascii_block(f);
            free(encryption);
            ek_ascii_to_bytes(s, &encryption, &encryptionlen);
            free(s);

            // This one triggers an actual test!
            printf("%s: ", example_name);

            int nbits = ek_bigint_nbits(modulus);

            int k = nbits / 8;
            uint8_t *enc = calloc(1, k);
            int enc_len; // will be set to k

            if (1) {
                enc_usecs -= utime_now();
                ek_rsa_ssa_pkcs_15_sign_sha1(modulus, privexp, message, messagelen, enc, &enc_len);
                enc_usecs += utime_now();
            }

//            ek_print_buffer(enc, enc_len);
//            ek_print_buffer(encryption, encryptionlen);

            if (enc_len == encryptionlen && !memcmp(enc, encryption, encryptionlen))
                printf("ENC OK\n");
            else {
                ek_bigint_print(modulus);
                ek_print_buffer(encryption, encryptionlen);
                ek_print_buffer(enc, encryptionlen);
                assert(0);
            }

            free(enc);

            nops++;

            goto cleanup;
        }

        printf("Unknown line: %s\n", buf);

      cleanup:
        continue;

    }

    fclose(f);

    printf("enc: %.5f ms,  dec_privexp: %.5f ms,  dec_crt: %.5f ms, nops %d\n",
           enc_usecs / 1000.0, dec_privexp_usecs / 1000.0, dec_crt_usecs / 1000.0, nops);
}
