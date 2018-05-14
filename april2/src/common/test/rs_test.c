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
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>

#include "common/time_util.h"
#include "rs.h"

static void generate_polys()
{
    for (int nbits = 1; nbits <= 16; nbits++) {
        for (int poly = (1 << nbits); poly < (2 << nbits); poly++) {
            printf("%d\r", poly);
            fflush(NULL);
            rs_t *rs = rs_create(nbits, poly, 8, 8); // datasz, encsz don't matter.
            int valid = rs_validate(rs);
            rs_destroy(rs);
            if (valid) {
                printf("%3d : 0x%04x\n", nbits, poly);
                break;
            }
        }
    }
}

static void print_mat(uint16_t *M, int nrows, int ncols)
{
    for (int i = 0; i < nrows; i++) {
        for (int j = 0; j < ncols; j++) {
            printf("%3d ", M[i*ncols + j]);
        }
        printf("\n");
    }
}

int main(int argc, char *argv[])
{
//    generate_polys();
//    return 0;

    uint16_t data[] = { 'H', 'e', 'l', 'l', 'o', ' ', 'E', 'd', 'w', 'i', 'n', '\n'};
//    uint16_t data[] = { 0, 1, 2, 3, 4, 5, 6, 7 };
    rs_t *rs = rs_create(8, 0, 11, 250);

//    rs_t *rs = rs_create(2, 0, 2, 4);
//    uint16_t data[] = { 3, 1 };

//    rs_t *rs = rs_create(3, 0x000b, 3, 6);
//    uint16_t data[] = { 2, 1, 6 };

    uint16_t enc[rs->encsz];

    assert(rs_validate(rs));
    rs_encode(rs, data, enc);

    int64_t utime0 = utime_now();

    if (1) {
        for (int trial = 0; trial < 1000; trial++) {

            // simulate reception of enc.
            int nrenc = rs->encsz;
            uint16_t rencidx[nrenc];

            // random permutation order
            for (int i = 0; i < nrenc; i++) {
              tryagain:
                rencidx[i] = random() % rs->encsz;
                for (int j = 0; j < i; j++) {
                    if (rencidx[i] == rencidx[j])
                        goto tryagain;
                }
            }

            uint16_t renc[nrenc];
            for (int i = 0; i < nrenc; i++) {
                renc[i] = enc[rencidx[i]];
            }

            // make reception errors
            int nerrors = trial / 10;

            for (int i = 0; i < nerrors; i++) {
                int pos = random() % nrenc;
                renc[pos] = random() & (rs->N - 1);
            }
            uint16_t rdata[rs->datasz];

            int res = rs_bw_decoder(rs, rencidx, nrenc, renc, rdata);

            printf("%5d [nerr = %4d, nz = %3d] ", trial, nerrors, res);

            if (res == 0) {
                for (int i = 0; i < rs->datasz; i++)
                    printf("%c", rdata[i]);
            }
            printf("\n");
            if (res)
                break;
        }
    }

    int64_t utime1 = utime_now();

    for (int iter = 0; iter < 1000; iter++) {
        // simulate reception of datasz random elements of enc.
        uint16_t rencidx[rs->datasz];

        for (int i = 0; i < rs->datasz; i++) {
          tryagain2:
            rencidx[i] = random() % rs->encsz;
            for (int j = 0; j < i; j++) {
                if (rencidx[i] == rencidx[j])
                    goto tryagain2;
            }
        }

//        print_mat(rencidx, 1, rs->datasz);
        uint16_t renc[rs->datasz];
        for (int i = 0; i < rs->datasz; i++) {
            renc[i] = enc[rencidx[i]];
        }

        rs_erasure_decoder_t *rsd = rs_erasure_decoder_create(rs, rencidx);
        uint16_t rdata[rs->datasz];
        rs_erasure_decoder_decode(rsd, renc, rdata);

        printf("%8d: ", iter);
        for (int i = 0; i < rs->datasz; i++)
            printf("%c", rdata[i]);
        printf("\n");

        if (memcmp(rdata, data, rs->datasz)) {
            printf("FAIL\n");
            exit(-1);
        }

        rs_erasure_decoder_destroy(rsd);
    }

    int64_t utime2 = utime_now();

    printf("%15.3f sec, %15.3f sec\n", (utime1 - utime0) / 1.0E6, (utime2 - utime1) / 1.0E6);
}
