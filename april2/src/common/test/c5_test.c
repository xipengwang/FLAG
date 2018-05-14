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
#include <string.h>
#include <assert.h>

#define NCOPIES 10000
#define LOG_LENGTH 24

#include "../c5.h"

static int irand(int max)
{
    assert(max != 0);

    return random() % max;
}

static int imin(int a, int b)
{
    return (a < b) ? a : b;
}

static void cnull(const uint8_t *_in, int _inlen, uint8_t *_out, int *_outlen)
{
    memcpy(_out, _in, _inlen);
    *_outlen = _inlen;
}

typedef void(*inout_t)(const uint8_t *in, int inlen, uint8_t *out, int *outlen);

struct code {
    char *name;
    inout_t compress;
    inout_t uncompress;
};

struct code codes[] = { { .name = "c5",   .compress = c5, .uncompress = uc5 },
//                        { .name = "c6",   .compress = c6, .uncompress = uc6 },
                        { .name = "null", .compress = cnull, .uncompress = cnull },
};

int main(int argc, char *argv[])
{
    int debug = 0;

    int ncodes = sizeof(codes) / sizeof(struct code);

    int PAD = C5_PAD; // maximum of any enabled codes

    for (int seed = 0; seed < 100; seed++) {
        srand(seed);

        if (debug && ((seed % 1000) == 0))
            printf("seed %d\n", seed);

        int inbuflen = (1 << irand(LOG_LENGTH));
        inbuflen += irand(inbuflen);
        if (seed == 0)
            inbuflen = 0;

        uint8_t *inbuf = malloc(inbuflen + PAD);

        // first, randomize the buffer... but not so random that it's uncompressable!
        if (inbuflen > 0) {
            int kmul = irand(255);
            for (int i = 0; i < inbuflen; i++)
                inbuf[i] = (kmul*random())&0xff;

            int ncopies = irand(NCOPIES);
            for (int i = 0; i < ncopies; i++) {
                int srcidx = irand(inbuflen);
                int destidx = irand(inbuflen);
                int copylen = (1 << irand(LOG_LENGTH));
                copylen += irand(copylen);
                int maxcopylen = imin(inbuflen - srcidx, inbuflen - destidx);
                if (copylen > maxcopylen)
                    copylen = maxcopylen;

                memmove(&inbuf[destidx], &inbuf[srcidx], copylen);
            }
        }

        // now test each code.
        for (int codeidx = 0; codeidx < ncodes; codeidx++) {
            int cbufalloc = inbuflen*2 + PAD + 1024;
            uint8_t *cbuf = malloc(cbufalloc);
            int cbuflen;
            codes[codeidx].compress(inbuf, inbuflen, cbuf, &cbuflen);
            for (int i = cbuflen; i < cbufalloc; i++)
                cbuf[i] = random()&0xff;

            int ucbufalloc = inbuflen + PAD + 1024;
            uint8_t *ucbuf = malloc(ucbufalloc);
            for (int i = 0; i < ucbufalloc; i++)
                ucbuf[i] = random() & 0xff;

            int ucbuflen;
            codes[codeidx].uncompress(cbuf, cbuflen, ucbuf, &ucbuflen);

            if (ucbuflen != inbuflen) {
                printf("FAIL length seed %d, ucbuflen %d, inbuflen %d\n", seed, ucbuflen, inbuflen);
                exit(-1);
            }

            if (memcmp(inbuf, ucbuf, inbuflen)) {
                printf("FAIL contents %d\n", seed);
                exit(-1);
            }

            if (debug)
                printf("%8s %8d %8d %15f\n", codes[codeidx].name, inbuflen,  cbuflen, 100.0*cbuflen / ucbuflen);
            free(cbuf);
            free(ucbuf);
        }
        free(inbuf);
    }
}
