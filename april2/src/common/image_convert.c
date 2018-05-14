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
#include <stdint.h>

#include "common/image_u8.h"
#include "common/image_u8x3.h"
#include "common/image_u8x4.h"

#include "lcmtypes/image_t.h"

#include "image_convert.h"

// Forward declaration
static image_u8x4_t *debayer_rggb_to_u8x4(const image_t *image);

////////////////////////////////////////////////////////////
// From image_u8

image_u8x3_t *image_u8_to_u8x3(const image_u8_t *in)
{
    image_u8x3_t *out = image_u8x3_create(in->width, in->height);

    for (int y = 0; y < in->height; y++) {
        for (int x = 0; x < in->width; x++) {

            uint8_t gray = in->buf[y*in->stride + x];

            out->buf[y*out->stride + x*3 + 0] = gray;
            out->buf[y*out->stride + x*3 + 1] = gray;
            out->buf[y*out->stride + x*3 + 2] = gray;
        }
    }

    return out;
}

image_u8x4_t *image_u8_to_u8x4(const image_u8_t *inp)
{
    const image_u8_t in = *inp;

    image_u8x4_t *outp = image_u8x4_create(in.width, in.height);
    image_u8x4_t out = *outp;

    for (int y = 0; y < in.height; y++) {
        for (int x = 0; x < in.width; x++) {

            uint8_t gray = in.buf[y*in.stride + x];

            out.buf[y*out.stride + 4*x + 0] = gray;
            out.buf[y*out.stride + 4*x + 1] = gray;
            out.buf[y*out.stride + 4*x + 2] = gray;
            out.buf[y*out.stride + 4*x + 3] = 0xff;
        }
    }

    return outp;
}

////////////////////////////////////////////////////////////
// From image_u8x3

image_u8_t *image_u8x3_to_u8(const image_u8x3_t *in)
{
    image_u8_t *out = image_u8_create(in->width, in->height);

    for (int y = 0; y < in->height; y++) {
        for (int x = 0; x < in->width; x++) {

            uint8_t r, g, b;
            r = in->buf[y*in->stride + x*3 + 0];
            g = in->buf[y*in->stride + x*3 + 1];
            b = in->buf[y*in->stride + x*3 + 2];

            uint8_t gray = (r + g + g + b) / 4;

            out->buf[y*out->stride + x] = gray;
        }
    }

    return out;
}

image_u8x4_t *image_u8x3_to_u8x4(const image_u8x3_t *inp)
{
    const image_u8x3_t in = *inp;

    image_u8x4_t *outp = image_u8x4_create(in.width, in.height);
    image_u8x4_t out = *outp;

    for (int y = 0; y < in.height; y++) {
        for (int x = 0; x < in.width; x++) {

            uint8_t a, r, g, b;
            r = in.buf[y*in.stride + 3*x + 0];
            g = in.buf[y*in.stride + 3*x + 1];
            b = in.buf[y*in.stride + 3*x + 2];
            a = 0xff;

            out.buf[y*out.stride + 4*x + 0] = r;
            out.buf[y*out.stride + 4*x + 1] = g;
            out.buf[y*out.stride + 4*x + 2] = b;
            out.buf[y*out.stride + 4*x + 3] = a;
        }
    }

    return outp;
}

////////////////////////////////////////////////////////////
// From image_u8x4

image_u8_t *image_u8x4_to_u8(const image_u8x4_t *inp)
{
    const image_u8x4_t in = *inp;

    image_u8_t *outp = image_u8_create(in.width, in.height);
    image_u8_t out = *outp;

    for (int y = 0; y < in.height; y++) {
        for (int x = 0; x < in.width; x++) {

            uint8_t r = in.buf[y*in.stride + 4*x + 0];
            uint8_t g = in.buf[y*in.stride + 4*x + 1];
            uint8_t b = in.buf[y*in.stride + 4*x + 2];

            uint8_t gray = (r + g + g + b) >> 2;

            out.buf[y*out.stride + x] = gray;
        }
    }

    return outp;
}

image_u8x3_t *image_u8x4_to_u8x3(const image_u8x4_t *inp)
{
    const image_u8x4_t in = *inp;

    image_u8x3_t *outp = image_u8x3_create(in.width, in.height);
    image_u8x3_t out = *outp;

    for (int y = 0; y < in.height; y++) {
        for (int x = 0; x < in.width; x++) {

            uint8_t r = in.buf[y*in.stride + 4*x + 0];
            uint8_t g = in.buf[y*in.stride + 4*x + 1];
            uint8_t b = in.buf[y*in.stride + 4*x + 2];

            out.buf[y*out.stride + 3*x + 0] = r;
            out.buf[y*out.stride + 3*x + 1] = g;
            out.buf[y*out.stride + 3*x + 2] = b;
        }
    }

    return outp;
}

////////////////////////////////////////////////////////////
// image_t wrappers

const image_t image_t_wrap_image_u8(const image_u8_t *in, int64_t utime)
{
    image_t out;
    out.utime       = utime;
    out.width       = in->width;
    out.height      = in->height;
    out.row_stride  = in->stride;
    out.pixelformat = IMAGE_T_PIXEL_FORMAT_GRAY;
    out.size        = in->stride*in->height;
    out.data        = (uint8_t*) in->buf;
    out.nmetadata   = 0;
    out.metadata    = NULL;
    return out;
}

const image_u8_t image_u8_wrap_image_t(const image_t *in)
{
    assert(in->pixelformat == IMAGE_T_PIXEL_FORMAT_GRAY);

    image_u8_t out = { .width = in->width, .height = in->height, .stride = in->row_stride, .buf = in->data };
    return out;
}

const image_t image_t_wrap_image_u8x3(const image_u8x3_t *in, int64_t utime)
{
    image_t out;
    out.utime       = utime;
    out.width       = in->width;
    out.height      = in->height;
    out.row_stride  = in->stride;
    out.pixelformat = IMAGE_T_PIXEL_FORMAT_RGB;
    out.size        = in->stride*in->height;
    out.data        = in->buf;
    out.nmetadata   = 0;
    out.metadata    = NULL;
    return out;
}

const image_u8x3_t image_u8x3_wrap_image_t(const image_t *in)
{
    assert(in->pixelformat == IMAGE_T_PIXEL_FORMAT_RGB);

    image_u8x3_t out = { .width = in->width, .height = in->height, .stride = in->row_stride, .buf = in->data };
    return out;
}

const image_t image_t_wrap_image_u8x4(const image_u8x4_t *in, int64_t utime)
{
    image_t out;
    out.utime       = utime;
    out.width       = in->width;
    out.height      = in->height;
    out.row_stride  = in->stride;
    out.pixelformat = IMAGE_T_PIXEL_FORMAT_RGBA;
    out.size        = in->stride*in->height;
    out.data        = in->buf;
    out.nmetadata   = 0;
    out.metadata    = NULL;
    return out;
}

const image_u8x4_t image_u8x4_wrap_image_t(const image_t *in)
{
    assert(in->pixelformat == IMAGE_T_PIXEL_FORMAT_RGBA);

    image_u8x4_t out = { .width = in->width, .height = in->height, .stride = in->row_stride, .buf = in->data };
    return out;
}

////////////////////////////////////////////////////////////
// Conversions for raw image types

static image_u8_t *debayer_rggb_to_u8(const image_t *in)
{
    image_u8x4_t *tmp = debayer_rggb_to_u8x4(in);
    image_u8_t *out = image_u8x4_to_u8(tmp);
    image_u8x4_destroy(tmp);
    return out;

}

image_u8_t *image_t_to_image_u8(const image_t *in)
{
    image_u8_t *out = NULL;

    switch (in->pixelformat)
    {
        case IMAGE_T_PIXEL_FORMAT_GRAY: {
            image_u8_t wrapper = image_u8_wrap_image_t(in);
            out = image_u8_copy(&wrapper);
            break;
        }

        case IMAGE_T_PIXEL_FORMAT_RGB: {
            image_u8x3_t wrapper = image_u8x3_wrap_image_t(in);
            out = image_u8x3_to_u8(&wrapper);
            return out;
        }

        case IMAGE_T_PIXEL_FORMAT_BAYER_RGGB: {
            out = debayer_rggb_to_u8(in);
            break;
        }
    }

    return out;
}

static image_u8x4_t *debayer_rggb_to_u8x4(const image_t *image)
{
    image_u8x4_t *im = image_u8x4_create(image->width, image->height);

    uint8_t *in  = (uint8_t*) image->data;
    uint8_t *out = (uint8_t*) im->buf;

    int width  = im->width;
    int height = im->height;
    int stride = im->stride;

    // loop over each 2x2 bayer block and compute the pixel values for each element.
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

            int idx = y*stride + 4*x;

            // top left pixel (R)
            r = (in[X11]);
            g = ((in[X01+width])+(in[X10+1])+(in[X11+1])+(in[X11+width])) / 4;
            b = ((in[X00+width+1])+(in[X10+width+1])+(in[X10+width+1])+(in[X11+width+1])) / 4;
            out[idx+0] = r;
            out[idx+1] = g;
            out[idx+2] = b;
            out[idx+3] = 0xff;

            // top right pixel (G)
            r = ((in[X11])+(in[X12])) / 2;
            g = (in[X11+1]);
            b = ((in[X01+width+1])+(in[X11+width+1])) / 2;
            out[idx+4] = r;
            out[idx+5] = g;
            out[idx+6] = b;
            out[idx+7] = 0xff;

            // bottom left pixel (G)
            r = ((in[X11])+(in[X21])) / 2;
            g = (in[X11+width]);
            b = ((in[X10+width+1])+(in[X11+width+1])) / 2;
            idx += stride;
            out[idx+0] = r;
            out[idx+1] = g;
            out[idx+2] = b;
            out[idx+3] = 0xff;

            // bottom right pixel (B)
            r = ((in[X11])+(in[X12])+(in[X21])+(in[X22])) / 4;
            g = ((in[X11+1])+(in[X11+width])+(in[X12+width])+(in[X21+1]))/ 4;
            b = (in[X11+width+1]);
            out[idx+4] = r;
            out[idx+5] = g;
            out[idx+6] = b;
            out[idx+7] = 0xff;
        }
    }

    return im;
}

static image_u8x4_t *debayer_gbrg_to_u8x4(const image_t *image)
{
    image_u8x4_t *im = image_u8x4_create(image->width, image->height);

    uint8_t *in  = (uint8_t*) image->data;
    uint8_t *out = (uint8_t*) im->buf;

    int width  = im->width;
    int height = im->height;
    int stride = im->stride;

    // loop over each 2x2 bayer block and compute the pixel values for each element.
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

            int idx = y*stride + 4*x;

            // top left pixel (G)
            r = ((in[X01+width])+(in[X11+width])) / 2;
            g = (in[X11]);
            b = ((in[X10+1])+(in[X11+1])) / 2;
            out[idx+0] = r;
            out[idx+1] = g;
            out[idx+2] = b;
            out[idx+3] = 0xff;

            // top right pixel (B)
            r = ((in[X01+width])+(in[X02+width])+(in[X11+width])+(in[X12+width])) / 4;
            g = ((in[X01+width+1])+(in[X11])+(in[X11+width+1])+(in[X12])) / 4;
            b = (in[X11+1]);
            out[idx+4] = r;
            out[idx+5] = g;
            out[idx+6] = b;
            out[idx+7] = 0xff;

            // bottom left pixel (R)
            r = (in[X11+width]);
            g = ((in[X10+width+1])+(in[X11])+(in[X11+width+1])+(in[X21])) / 4;
            b = ((in[X10+1])+(in[X11+1])+(in[X20+1])+(in[X21+1])) / 4;
            idx += stride;
            out[idx+0] = r;
            out[idx+1] = g;
            out[idx+2] = b;
            out[idx+3] = 0xff;

            // bottom right pixel (G)
            r = ((in[X11+width])+(in[X12+width])) / 2;
            g = (in[X11+width+1]);
            b = ((in[X11+1])+(in[X21+1])) / 2;
            out[idx+4] = r;
            out[idx+5] = g;
            out[idx+6] = b;
            out[idx+7] = 0xff;
        }
    }

    return im;
}

static image_u8x4_t *be_gray16_to_u8x4(const image_t *in)
{
    assert(in->pixelformat == IMAGE_T_PIXEL_FORMAT_BE_GRAY16);
    image_u8x4_t *out = image_u8x4_create(in->width, in->height);

    assert(in->height*in->row_stride == in->size);

    for (int y = 0; y < out->height; y++) {
        for (int x = 0; x < out->width; x++) {
            //uint16_t val16 = (in->data[y*in->row_stride + 2*x + 0] << 2) + (in->data[y*in->row_stride + 2*x + 1] >> 6);
            //val16 = val16 >> 2;
            uint16_t val16 = (in->data[y*in->row_stride + 2*x + 0]);
            uint8_t val8 = val16 & 0xff;
            out->buf[y*out->stride + 4*x + 0] = val8;
            out->buf[y*out->stride + 4*x + 1] = val8;
            out->buf[y*out->stride + 4*x + 2] = val8;
            out->buf[y*out->stride + 4*x + 3] = 0xff;
        }
    }

    return out;
}


static image_u8x4_t *debayer_rggb_to_u8x4_half(const image_t *inp)
{
    const image_t in = *inp;
    int stride = in.row_stride;

    image_u8x4_t *outp = image_u8x4_create(in.width/2, in.height/2);
    image_u8x4_t out = *outp;

    // loop over each 2x2 bayer block and compute the pixel values for each element.
    for (int y = 0; y < out.height; y++) {
        for (int x = 0; x < out.width; x++) {

            // TODO try interpolating pixel value instead of pretending that pixels overlap

            out.buf[y*out.stride + 4*x + 0] = in.data[(2*y+0)*stride + (2*x+0)];
            out.buf[y*out.stride + 4*x + 1] = (in.data[(2*y+0)*stride + (2*x+1)] + in.data[(2*y+1)*stride + (2*x+0)]) >> 1;
            out.buf[y*out.stride + 4*x + 2] = in.data[(2*y+1)*stride + (2*x+1)];
            out.buf[y*out.stride + 4*x + 3] = 0xff;
        }
    }

    return outp;
}

static image_u8x4_t *debayer_gbrg_to_u8x4_half(const image_t *inp)
{
    const image_t in = *inp;
    int stride = in.row_stride;

    image_u8x4_t *outp = image_u8x4_create(in.width/2, in.height/2);
    image_u8x4_t out = *outp;

    // loop over each 2x2 bayer block and compute the pixel values for each element.
    for (int y = 0; y < out.height; y++) {
        for (int x = 0; x < out.width; x++) {

            // TODO try interpolating pixel value instead of pretending that pixels overlap

            out.buf[y*out.stride + 4*x + 0] = in.data[(2*y+1)*stride + (2*x+0)];
            out.buf[y*out.stride + 4*x + 1] = (in.data[(2*y+0)*stride + (2*x+0)] + in.data[(2*y+1)*stride + (2*x+1)]) >> 1;
            out.buf[y*out.stride + 4*x + 2] = in.data[(2*y+0)*stride + (2*x+1)];
            out.buf[y*out.stride + 4*x + 3] = 0xff;
        }
    }

    return outp;
}

image_u8x4_t *image_t_to_image_u8x4(const image_t *in)
{
    image_u8x4_t *out = NULL;

    switch (in->pixelformat)
    {
        case IMAGE_T_PIXEL_FORMAT_BE_GRAY16: {
            out = be_gray16_to_u8x4(in);
            break;
        }
        case IMAGE_T_PIXEL_FORMAT_GRAY: {
            image_u8_t wrapper = image_u8_wrap_image_t(in);
            out = image_u8_to_u8x4(&wrapper);
            break;
        }

        case IMAGE_T_PIXEL_FORMAT_RGB: {
            image_u8x3_t wrapper = image_u8x3_wrap_image_t(in);
            out = image_u8x3_to_u8x4(&wrapper);
            return out;
        }

        case IMAGE_T_PIXEL_FORMAT_BAYER_RGGB: {
            out = debayer_rggb_to_u8x4(in);
            break;
        }

        case IMAGE_T_PIXEL_FORMAT_BAYER_GBRG: {
            out = debayer_gbrg_to_u8x4(in);
            break;
        }
        default: {
            printf("UKNOWN IMAGE FORMAT %d\n", in->pixelformat);
            assert(0);
        }
    }

    return out;
}

image_u8x4_t *image_t_to_image_u8x4_half(const image_t *in)
{
    image_u8x4_t *out = NULL;

    switch (in->pixelformat)
    {
        case IMAGE_T_PIXEL_FORMAT_BAYER_RGGB: {
            out = debayer_rggb_to_u8x4_half(in);
            break;
        }

        case IMAGE_T_PIXEL_FORMAT_BAYER_GBRG: {
            out = debayer_gbrg_to_u8x4_half(in);
            break;
        }
    }

    return out;
}
