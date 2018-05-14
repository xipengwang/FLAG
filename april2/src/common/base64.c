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

#include <assert.h>
#include <string.h>
#include "base64.h"
#include <stdlib.h>

// convert a base64 character into a six bit number.
// sets *err if bad character is found.
static uint8_t base64_decode_char(uint8_t c, int *err)
{
    if (c >= 'A' && c <= 'Z')
        return c - 'A' + 0;
    if (c >= 'a' && c <= 'z')
        return c - 'a' + 26;
    if (c >= '0' && c <= '9')
        return c - '0' + 52;
    if (c == '+')
        return 62;
    if (c == '/')
        return 63;
    if (c == '=')
        return 0;

    *err = 1;
    return 0;
}

// convert a six bit number into a base64 character.
static uint8_t base64_encode_char(uint8_t c)
{
    if (c < 26)
        return 'A' + c;
    if (c < 52)
        return 'a' + c - 26;
    if (c < 62)
        return '0' + c - 52;
    if (c == 62)
        return '+';
    if (c == 63)
        return '/';

    assert(0);
    return '?';
}

// out should be allocated to be "long enough" (actual output length
// plus 3; strlen(in)+3 will do). Returns 0 on success.
int base64_decode(const void *_in, void *_out, int *outlen)
{
    const uint8_t *in = _in;
    uint8_t *out = _out;

    int inlen = strlen((char*) in);
    int inpos = 0;
    int outpos = 0;

    int err = 0;

    for (int inpos = 0; inpos + 4 <= inlen; inpos += 4) {

        // a5 a4 a3 a2 a1 a0 b5 b4 | b3 b2 b1 b0 c5 c4 c3 c2 | c1 c0 d5 d4 d3 d2 d1 d0
        uint8_t a = base64_decode_char(in[inpos+0], &err);
        uint8_t b = base64_decode_char(in[inpos+1], &err);
        uint8_t c = base64_decode_char(in[inpos+2], &err);
        uint8_t d = base64_decode_char(in[inpos+3], &err);

        if (in[inpos+0] != '=') {
            out[outpos++] = (a<<2) + (b>>4);
            if (in[inpos+2] != '=') {
                out[outpos++] = (b<<4) + (c>>2);
                if (in[inpos+3] != '=')
                    out[outpos++] = (c<<6) + (d<<0);
            }
        }
    }

    // provide handy NULL termination.
    out[outpos] = 0;

    *outlen = outpos;
    if (err)
        return -1;
    return 0;
}

char *base64_encode_alloc(const void *_in, int inlen)
{
    // turn every 3 bytes into 4 bytes.
    int outlen = (inlen + 2) / 3 * 4;
    char *b64 = malloc(outlen + 1);
    int chk_outlen;
    int res = base64_encode(_in, inlen, b64, &chk_outlen);
    assert (res == 0);
    assert(chk_outlen == outlen);
    return b64;
}

int base64_encode(const void *_in, int inlen, void *_out, int *outlen)
{
    const uint8_t *in = _in;
    uint8_t *out = _out;

    int outpos = 0;

/*    if (inlen == 0) {
        out[outpos++] = '=';
        out[outpos++] = '=';
        out[outpos++] = '=';
        out[outpos++] = '=';

    } else {
*/
    if (1) {
        for (int inpos = 0; inpos < inlen; inpos += 3) {
            // x7 x6 x5 x4 x3 x2 | x1 x0 y7 y6 y5 y4 | y3 y2 y1 y0 z7 z6 | z5 z4 z3 z2 z1 z0
            uint8_t x = in[inpos+0];
            uint8_t y = inpos + 1 < inlen ? in[inpos+1] : 0;
            uint8_t z = inpos + 2 < inlen ? in[inpos+2] : 0;

            uint8_t a = (x>>2) & 0x3f;
            uint8_t b = ((x<<4) + (y>>4)) & 0x3f;
            uint8_t c = ((y<<2) + (z>>6)) & 0x3f;
            uint8_t d = z & 0x3f;

            out[outpos++] = base64_encode_char(a);
            out[outpos++] = base64_encode_char(b);
            out[outpos++] = inpos + 1 < inlen ? base64_encode_char(c) : '=';
            out[outpos++] = inpos + 2 < inlen ? base64_encode_char(d) : '=';
        }
    }

    // handy NULL termination
    out[outpos] = 0;

    *outlen = outpos;
    return 0;
}
