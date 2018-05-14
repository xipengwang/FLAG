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

#include "ek/ek.h"

void poor_random_buffer(void *buf, int len)
{
    for (int i = 0; i < len; i++) {
        ((uint8_t*) buf)[i] = random();
    }
    return;

    FILE *f = fopen("/dev/urandom", "r");

    if (fread(buf, 1, len,  f) != len)
        exit(-1); // fatal!

    fclose(f);
}

void bc_test(ek_bigint_t *a, const char *op, ek_bigint_t *b, ek_bigint_t *r)
{
    FILE *f = fopen("/tmp/bc.tmp", "w");
    fprintf(f, "obase=16\nibase=16\n");

    char *as = ek_bigint_to_string_hex(a);
    char *bs = ek_bigint_to_string_hex(b);
    char *rs = ek_bigint_to_string_hex(r);

    fprintf(f, "(%s %s %s) == %s\n", as, op, bs, rs);
    fclose(f);

    free(as);
    free(bs);
    free(rs);

    FILE *p = popen("bc -q < /tmp/bc.tmp", "r");
    char line[1024];
    fgets(line, sizeof(line), p);
    fclose(p);

    printf("%s ", op);
    fflush(NULL);

    assert (line[0] == '1');
}

void bc_test_op(const char *tag, const char *fmt, ek_bigint_t *bigints[])
{
    FILE *f = fopen("/tmp/bc.tmp", "w");
    fprintf(f, "obase=16\nibase=16\n");

    int bigint_pos = 0;

    for (int i = 0; i < strlen(fmt); i++) {
        char c = fmt[i];

        if (c == 'I') {
            char *s = ek_bigint_to_string_hex(bigints[bigint_pos]);
            bigint_pos++;
            fprintf(f, "%s", s);
            free(s);
        } else {
            fprintf(f, "%c", c);
        }
    }
    fprintf(f, "\n");
    fclose(f);

    FILE *p = popen("bc -q < /tmp/bc.tmp", "r");
    char line[1024];
    fgets(line, sizeof(line), p);
    fclose(p);

    printf("%s ", tag);
    fflush(NULL);

    assert (line[0] == '1');
}

int main(int argc, char *argv[])
{
    ek_bigint_t *zero = ek_bigint_create_int(0);
    ek_bigint_t *one = ek_bigint_create_int(1);
    ek_bigint_t *two = ek_bigint_create_int(2);

    if (0) {
        ek_bigint_t *a = ek_bigint_create_int(1);
        ek_bigint_t *b = ek_bigint_create_int(2);

        for (int i = 0; i < 4096; i++) {
            assert(ek_bigint_equal(a, a));
            assert(!ek_bigint_equal(b, a));
            assert(ek_bigint_less_than(a, b));
            assert(!ek_bigint_less_than(b, a));

            a = ek_bigint_mul(a, two);
            b = ek_bigint_mul(b, two);
        }
    }

    while (1) {
        ek_bigint_t *a, *b, *mod;

        if (1) {
            int rblen = (random() & 31) + 1;
            uint8_t *rb = malloc(rblen);
            poor_random_buffer(rb, rblen);
            a = ek_bigint_create_from_bytes(rb, rblen);
            free(rb);
        }

        if (1) {
            int rblen = (random() & 31) + 1;
            uint8_t *rb = malloc(rblen);
            poor_random_buffer(rb, rblen);
            b = ek_bigint_create_from_bytes(rb, rblen);
            free(rb);
        }

        if (1) {
            int rblen = (random() & 31) + 1;
            uint8_t *rb = malloc(rblen);
            poor_random_buffer(rb, rblen);
            mod = ek_bigint_create_from_bytes(rb, rblen);
            free(rb);
        }

        if (1) {
            ek_bigint_ensure_nwords(a, random() & 31);
        }

        switch (random() % 8) {
            case 0: {
                ek_bigint_t *r = ek_bigint_copy(a);
                ek_bigint_add_inplace(r, b);
                bc_test_op("+", "(I + I) == I", (ek_bigint_t*[]) { a, b, r });
                ek_bigint_destroy(r);
                break;
            }

            case 1: {
                if (ek_bigint_less_than(a, b)) {
                    ek_bigint_t *r = ek_bigint_copy(b);
                    ek_bigint_sub_inplace(r, a);
                    bc_test_op("-", "(I - I) == I", (ek_bigint_t*[]) { b, a, r });
                    ek_bigint_destroy(r);
                } else {
                    ek_bigint_t *r = ek_bigint_copy(a);
                    ek_bigint_sub_inplace(r, b);
                    bc_test_op("-", "(I - I) == I", (ek_bigint_t*[]) { a, b, r });
                    ek_bigint_destroy(r);
                }
                break;
            }

            case 2: {
                if (!ek_bigint_is_zero(b)) {
                    ek_bigint_t *r = ek_bigint_mod(a, b);
                    bc_test_op("%", "(I % I) == I", (ek_bigint_t*[]) { a, b, r });
                    ek_bigint_destroy(r);
                }
                break;
            }

            case 3: {
                ek_bigint_t *r = ek_bigint_create_int(ek_bigint_less_than(a, b));
                bc_test_op("<", "(I < I) == I", (ek_bigint_t*[]) { a, b, r });
                ek_bigint_destroy(r);
                break;
            }

            case 4: {
                ek_bigint_t *r = ek_bigint_create_int(ek_bigint_less_equal(a, b));
                bc_test_op("<=", "(I <= I) == I", (ek_bigint_t*[]) { a, b, r });
                ek_bigint_destroy(r);
                break;
            }

            case 5: {
                ek_bigint_t *r = ek_bigint_copy(a);
                ek_bigint_mul_2_inplace(r);
                bc_test_op("*2", "(I*I)==I", (ek_bigint_t*[]) { a, two, r });
                ek_bigint_destroy(r);
                break;
            }

            case 6: {
                ek_bigint_t *r = ek_bigint_copy(a);
                ek_bigint_div_2_inplace(r);

                bc_test_op("/2", "(I/I)==I", (ek_bigint_t*[]) { a, two, r });
                ek_bigint_destroy(r);
                break;
            }

            case 7: {
                if (!ek_bigint_is_zero(mod)) {
                    ek_bigint_t *am = ek_bigint_mod(a, mod);
                    ek_bigint_t *bm = ek_bigint_mod(b, mod);
                    ek_bigint_destroy(a);
                    ek_bigint_destroy(b);
                    a = am;
                    b = bm;

                    ek_bigint_t *r = ek_bigint_mul_mod(a, b, mod);
                    bc_test_op("*%", "((I*I)%I) == I", (ek_bigint_t*[]) { a, b, mod, r });
                    ek_bigint_div_2_inplace(r);
                    ek_bigint_destroy(r);
                }
                break;
            }

        }

        ek_bigint_destroy(a);
        ek_bigint_destroy(b);
        ek_bigint_destroy(mod);
    }

    printf("passed\n");
    return 0;
}
