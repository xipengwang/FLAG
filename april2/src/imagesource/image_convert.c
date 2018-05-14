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

#include "image_convert.h"

#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>

#include "common/image_convert.h"


////////////////////////////////////////////
// Guide to interpreting pixel formats
//
// U32  : byte-ordered R, G, B, A  (see image_u32.h)
//
// BGRA : byte-ordered B, G, R, A  (as used in 32BGRA on MacOS/iOS)
//
// RGBA : Format is byte-ordered, R, G, B, A.
//
// RGB / RGB24 / u8x3 : Format is byte-ordered, R, G, B
//
static inline int clamp(int v)
{
    if (v < 0)
        return 0;
    if (v > 255)
        return 255;
    return v;
}

// byte-order B, G, R, A ==> R, G, B
static image_u8x3_t *convert_bgra_to_u8x3(image_source_data_t *frmd)
{
    image_u8x3_t *im = image_u8x3_create(frmd->ifmt.width,
                                         frmd->ifmt.height);

    int width = im->width;
    int height = im->height;
    int stride = im->stride;

    uint8_t *out = (uint8_t*) im->buf;
    uint8_t *in  = (uint8_t*) frmd->data;

    for (int y = 0; y < height; y++) {
        int inoffset = 4*y*width;
        int outoffset = y*stride;

        for (int x = 0; x < width; x++) {
            int b = in[inoffset + 0];
            int g = in[inoffset + 1];
            int r = in[inoffset + 2];

            out[outoffset + 0] = r;
            out[outoffset + 1] = g;
            out[outoffset + 2] = b;

            inoffset += 4;
            outoffset += 3;
        }
    }

    return im;
}


static image_u8x3_t *convert_gray_to_u8x3(image_source_data_t *frmd)
{
    image_u8x3_t *im = image_u8x3_create(frmd->ifmt.width,
                                         frmd->ifmt.height);

    int width = im->width;
    int height = im->height;
    int stride = im->stride;

    uint8_t *out = (uint8_t*) im->buf;
    uint8_t *in  = (uint8_t*) frmd->data;

    for (int y = 0; y < height; y++) {
        int inoffset = y*width;
        int outoffset = y*stride;

        for (int x = 0; x < width; x++) {
            int b = in[inoffset];
            int g = in[inoffset];
            int r = in[inoffset];

            out[outoffset + 0] = r;
            out[outoffset + 1] = g;
            out[outoffset + 2] = b;

            inoffset += 1;
            outoffset += 3;
        }
    }

    return im;
}

static image_u8x3_t *convert_rgba_to_u8x3(image_source_data_t *frmd)
{
    image_u8x3_t *im = image_u8x3_create(frmd->ifmt.width,
                                         frmd->ifmt.height);

    int width = im->width;
    int height = im->height;
    int stride = im->stride;

    uint8_t *out = (uint8_t*) im->buf;
    uint8_t *in  = (uint8_t*) frmd->data;

    for (int y = 0; y < height; y++) {
        int inoffset = 4*y*width;
        int outoffset = y*stride;

        for (int x = 0; x < width; x++) {
            int r = in[inoffset + 0];
            int g = in[inoffset + 1];
            int b = in[inoffset + 2];

            out[outoffset + 0] = r;
            out[outoffset + 1] = g;
            out[outoffset + 2] = b;

            inoffset += 4;
            outoffset += 3;
        }
    }

    return im;
}

static image_u8x3_t *convert_rgb_to_u8x3(image_source_data_t *frmd)
{
    image_u8x3_t *im = image_u8x3_create(frmd->ifmt.width,
                                         frmd->ifmt.height);

    int width = im->width;
    int height = im->height;
    int stride = im->stride;

    uint8_t *out = (uint8_t*) im->buf;
    uint8_t *in  = (uint8_t*) frmd->data;

    for (int y = 0; y < height; y++) {
        memcpy(&out[y*stride], &in[3*y*width], 3*width);
    }

    return im;
}

static image_u8x3_t *convert_yuyv_to_u8x3(image_source_data_t *frmd)
{
    image_u8x3_t *im = image_u8x3_create(frmd->ifmt.width,
                                         frmd->ifmt.height);

    int width = im->width;
    int height = im->height;
    int stride = im->stride;

    int sstride = width*2;
    uint8_t *yuyv = (uint8_t*)(frmd->data);
    uint8_t *out = (uint8_t*) im->buf;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width / 2; x++) {

            int y1 = yuyv[y*sstride + 4*x+0]&0xff;
            int u  = yuyv[y*sstride + 4*x+1]&0xff;
            int y2 = yuyv[y*sstride + 4*x+2]&0xff;
            int v  = yuyv[y*sstride + 4*x+3]&0xff;

            int cb = ((u-128) * 454)>>8;
            int cr = ((v-128) * 359)>>8;
            int cg = ((v-128) * 183 + (u-128) * 88)>>8;
            int r, g, b;

            r = clamp(y1 + cr);
            b = clamp(y1 + cb);
            g = clamp(y1 - cg);

            out[y*stride+6*x + 0] = r;
            out[y*stride+6*x + 1] = g;
            out[y*stride+6*x + 2] = b;

            r = clamp(y2 + cr);
            b = clamp(y2 + cb);
            g = clamp(y2 - cg);

            out[y*stride+6*x + 3] = r;
            out[y*stride+6*x + 4] = g;
            out[y*stride+6*x + 5] = b;
        }
    }
    return im;
}

static image_u8x3_t *debayer_gbrg_to_u8x3(image_source_data_t *frmd)
{
    image_u8x3_t *im = image_u8x3_create(frmd->ifmt.width,
                                         frmd->ifmt.height);

    uint8_t *in = (uint8_t*) frmd->data;
    uint8_t *out = (uint8_t*) im->buf;

    int width = im->width;
    int height = im->height;
    int stride = im->stride;

    // Loop over each 2x2 bayer block and compute the pixel values for
    // each element
    for (int y = 0; y < height; y+=2) {
        for (int x = 0; x < width; x+=2) {
            int r = 0, g = 0, b = 0;

            // compute indices into bayer pattern for the nine 2x2 blocks we'll use.
            int X00 = (y-2)*width+(x-2);
            int X01 = (y-2)*width+(x+0);
            int X02 = (y-2)*width+(x+2);
            int X10 = (y+0)*width+(x-2);
            int X11 = (y+0)*width+(x+0);
            int X12 = (y+0)*width+(x+2);
            int X20 = (y+2)*width+(x-2);
            int X21 = (y+2)*width+(x+0);
            int X22 = (y+2)*width+(x+2);

            // handle the edges of the screen.
            if (y < 2) {
                X00 += 2*width;
                X01 += 2*width;
                X02 += 2*width;
            }
            if (y+2 >= height) {
                X20 -= 2*width;
                X21 -= 2*width;
                X22 -= 2*width;
            }
            if (x < 2) {
                X00 += 2;
                X10 += 2;
                X20 += 2;
            }
            if (x+2 >= width) {
                X02 -= 2;
                X12 -= 2;
                X22 -= 2;
            }

            int idx = y*stride + 3*x;

            // top left pixel (G)
            r = ((in[X01+width]) + (in[X11+width])) / 2;
            g = in[X11];
            b = ((in[X10+1]) + (in[X11+1])) / 2;
            out[idx+0] = r;
            out[idx+1] = g;
            out[idx+2] = b;

            // top right pixel (B)
            r = ((in[X01+width])+(in[X02+width])+(in[X01+width]) + (in[X12+width])) / 4;
            g = ((in[X01+width+1])+(in[X11])+(in[X12])+(in[X11+width+1])) / 4;
            b = (in[X11+1]);
            out[idx+3] = r;
            out[idx+4] = g;
            out[idx+5] = b;

            // bottom left pixel (R)
            r = (in[X11+width]);
            g = ((in[X11])+(in[X10+width+1])+(in[X11+width+1])+(in[X21])) / 4;
            b = ((in[X10+1])+(in[X11+1])+(in[X20+1])+(in[X21+1])) / 4;

            idx += stride;
            out[idx+0] = r;
            out[idx+1] = g;
            out[idx+2] = b;

            // bottom right pixel (G)
            r = ((in[X11+width])+(in[X12+width])) / 2;
            g = (in[X11+width+1]);
            b = ((in[X11+1])+(in[X21+1])) / 2;
            out[idx+3] = r;
            out[idx+4] = g;
            out[idx+5] = b;
        }
    }

    return im;
}

static int image_convert_u8x3_slow_warned = 0;

image_u8x3_t *image_convert_u8x3(image_source_data_t *isdata)
{
    if (!strcmp("BAYER_GBRG", isdata->ifmt.format)) {
        return debayer_gbrg_to_u8x3(isdata);

    } else if (!strcmp("YUYV", isdata->ifmt.format)) {
        return convert_yuyv_to_u8x3(isdata);

    } else if (!strcmp("BGRA", isdata->ifmt.format)) {
        return convert_bgra_to_u8x3(isdata);

    } else if (!strcmp("RGBA", isdata->ifmt.format)) {
        return convert_rgba_to_u8x3(isdata);

    } else if (!strcmp("RGB", isdata->ifmt.format)) {
        return convert_rgb_to_u8x3(isdata);

    } else if (!strcmp("GRAY", isdata->ifmt.format)) {
        return convert_gray_to_u8x3(isdata);
    }

    printf("Cannot convert %s to u8x3\n", isdata->ifmt.format);

    return NULL;
}

/* Useful for apriltag detector */
image_u8_t *image_convert_u8(image_source_data_t *isdata)
{
    // Check if image source is color
    image_u8x3_t *im_color = image_convert_u8x3(isdata);
    if (im_color != NULL) {
        image_u8_t *im_gray = image_u8x3_to_u8(im_color);
        image_u8x3_destroy(im_color);
        return im_gray;
    }

    if (!strcmp("GRAY", isdata->ifmt.format) ||
        !strcmp("GRAY8", isdata->ifmt.format)) {
        int width = isdata->ifmt.width;
        int height = isdata->ifmt.height;

        image_u8_t *im = image_u8_create(width, height);
        uint8_t *in = isdata->data;
        uint8_t *out = im->buf;
        for (int y = 0; y < height; y += 1) {
            for (int x = 0; x < width; x += 1) {
                out[y*im->stride + x] = in[y*width + x];
            }
        }

        return im;
    } else if (!strcmp("GRAY16", isdata->ifmt.format)) {
        int width = isdata->ifmt.width;
        int height = isdata->ifmt.height;

        image_u8_t *im = image_u8_create(width, height);
        uint16_t *in = isdata->data;
        uint8_t *out = im->buf;
        for (int y = 0; y < height; y += 1) {
            for (int x = 0; x < width; x += 1) {
                out[y*im->stride + x] = in[y*width + x] >> 8;
            }
        }

        return im;
    }

    return NULL;
}
