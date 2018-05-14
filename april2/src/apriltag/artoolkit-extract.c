/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.

This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

#include <stdio.h>

#include <stdint.h>
#include <inttypes.h>

#include "common/image_u8.h"

int main(int argc, char *argv[])
{
    image_u8_t *im = image_u8_create_from_pnm("artoolkit_markers.pnm");

    // black border around every tag
    int bb = 3;

    // white border (around every tag)
    int wb = 2;

    // image border
    int ib = 2;

    // bit dimension
    int d = 6;

    // how many tags across?
    int tw = (im->width - ib*2) / (2*bb+2*wb+d);
    int th = (im->height - ib*2) / (2*bb+2*wb+d);

    printf("%d x %d\n", tw, th);

    for (int ty = 0; ty < th; ty++) {
        for (int tx = 0; tx < tw; tx++) {

            uint64_t rcode = 0;

            for (int iy = 0; iy < d; iy++) {
                for (int ix = 0; ix < d; ix++) {
                    int y = ib + (2*bb+2*wb+d)*ty + wb + bb + iy;
                    int x = ib + (2*bb+2*wb+d)*tx + wb + bb + ix;

                    uint8_t v = im->buf[y*im->stride + x] > 128;

                    rcode = (rcode << 1) | v;
                }
            }

            printf("    tf->codes[%d] = 0x%012"PRIx64"UL;\n", ty*tw+tx, rcode);
        }
    }
}
