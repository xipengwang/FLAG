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
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "camera_math.h"
#include "rasterizer.h"


typedef struct {
    rasterizer_t super; // KEEP ME FIRST

    int iw, ih, ow, oh;
    int *indices; // maps output idx -> input idx, or -1 if invalid
} rasterizer_nn_t;

static image_u8_t *rasterize_u8_nn(rasterizer_t *_r, image_u8_t *in)
{
    rasterizer_nn_t *r = (rasterizer_nn_t *)_r;

    int iw = in->width;
    int ih = in->height;
    int ow = r->ow;
    int oh = r->oh;

    assert(iw == r->iw && ih == r->ih);

    image_u8_t *out = image_u8_create(ow, oh);
    int len = ow * oh;
    for (int i = 0; i < len; i += 1) {
        int idx = r->indices[i];
        if (idx == -1)
            continue;

        // align indexes to stride width
        int ox = i % ow;
        int oy = i / ow;
        int ix = idx % iw;
        int iy = idx / iw;

        out->buf[oy*out->stride + ox] = in->buf[iy*in->stride + ix];
    }

    return out;
}

static void destroy_nn(rasterizer_t *_r)
{
    rasterizer_nn_t *r = (rasterizer_nn_t *)_r;

    free(r->indices);
    free(r);
}

/* Maps an input view to output view using nearest neighbor rasterization */
rasterizer_t *rasterizer_create_nn(view_t *in, view_t* out)
{
    int iw = in->get_width(in);
    int ih = in->get_height(in);
    int ow = out->get_width(out);
    int oh = out->get_height(out);

    rasterizer_nn_t *r = calloc(1, sizeof(rasterizer_nn_t));

    r->iw = iw;
    r->ih = ih;
    r->ow = ow;
    r->oh = oh;
    r->indices = malloc(ow * oh * sizeof(*r->indices));
    r->super.rasterize_u8 = rasterize_u8_nn;
    r->super.destroy = destroy_nn;

    // Build lookup table
    for (int y = 0; y < oh; y += 1) {
        for (int x = 0; x < ow; x += 1) {
            matd_t *xy = matd_create_data(2, 1, (double[]){x, y});
            matd_t *_ray = out->pixels_to_ray(out, xy);
            matd_t *ray = snap_ray_to_plane(_ray);

            matd_t *ixy = in->ray_to_pixels(in, ray);
            int ix = (int)round(ixy->data[0]);
            int iy = (int)round(ixy->data[1]);

            int idx = -1;
            if (ix >= 0 && ix < iw && iy >= 0 && iy < ih)
                idx = iy*iw + ix;
            r->indices[y*ow + x] = idx;

            matd_destroy(xy);
            matd_destroy(_ray);
            matd_destroy(ray);
            matd_destroy(ixy);
        }
    }

    return (rasterizer_t *)r;
}


typedef struct {
    rasterizer_t super; // KEEP ME FIRST

    int iw, ih, ow, oh;
    int *indices;       // maps output idx -> input idx, or -1 if invalid
    int32_t *weights;   // fixed point weight
} rasterizer_bilinear_t;

static image_u8_t *rasterize_u8_bilinear(rasterizer_t *_r, image_u8_t *in)
{
    rasterizer_bilinear_t *r = (rasterizer_bilinear_t *)_r;

    int iw = in->width;
    int ih = in->height;
    int ow = r->ow;
    int oh = r->oh;

    assert(iw == r->iw && ih == r->ih);

    image_u8_t *out = image_u8_create(ow, oh);
    int len = ow * oh;
    for (int i = 0; i < len; i += 1) {
        int idx = r->indices[i];
        if (idx == -1)
            continue;

        // align indexes to stride width
        int ox = i % ow;
        int oy = i / ow;
        int ix = idx % iw;
        int iy = idx / iw;

        int32_t v00 = in->buf[iy*in->stride + ix] * r->weights[4*i];
        int32_t v10 = in->buf[iy*in->stride + ix+1]  * r->weights[4*i + 1];
        int32_t v01 = in->buf[(iy+1)*in->stride + ix]  * r->weights[4*i + 2];
        int32_t v11 = in->buf[(iy+1)*in->stride + ix+1]  * r->weights[4*i + 3];
        int32_t b = (v00 + v10 + v01 + v11) >> 16;

        out->buf[oy*out->stride + ox] = b;
    }

    return out;
}

static void destroy_bilinear(rasterizer_t *_r)
{
    rasterizer_bilinear_t *r = (rasterizer_bilinear_t *)_r;

    free(r->indices);
    free(r->weights);
    free(r);
}

rasterizer_t *rasterizer_create_bilinear(view_t *in, view_t* out)
{
    int iw = in->get_width(in);
    int ih = in->get_height(in);
    int ow = out->get_width(out);
    int oh = out->get_height(out);

    rasterizer_bilinear_t *r = calloc(1, sizeof(rasterizer_bilinear_t));

    r->iw = iw;
    r->ih = ih;
    r->ow = ow;
    r->oh = oh;
    r->indices = malloc(ow * oh * sizeof(*r->indices));
    r->weights = malloc(4 * ow * oh * sizeof(*r->weights));
    r->super.rasterize_u8 = rasterize_u8_bilinear;
    r->super.destroy = destroy_bilinear;

    // Build lookup table
    for (int y = 0; y < oh; y += 1) {
        for (int x = 0; x < ow; x += 1) {
            matd_t *xy = matd_create_data(2, 1, (double[]){x, y});
            matd_t *_ray = out->pixels_to_ray(out, xy);
            matd_t *ray = snap_ray_to_plane(_ray);

            matd_t *ixy = in->ray_to_pixels(in, ray);
            int ix = (int)ixy->data[0];
            int iy = (int)ixy->data[1];

            int idx = -1;
            if (ix >= 0 && ix+1 < iw && iy >= 0 && iy+1 < ih)
                idx = iy*iw + ix;
            r->indices[y*ow + x] = idx;

            // Bilinear weights
            if (idx != -1) {
                double dx = ixy->data[0] - ix;
                double dy = ixy->data[1] - iy;

                int offset = 4 * (y*ow + x);
                r->weights[offset+0] = (int32_t)((1-dx)*(1-dy) * (double)(1<<16));
                r->weights[offset+1] = (int32_t)(dx*(1-dy) * (double)(1<<16));
                r->weights[offset+2] = (int32_t)((1-dx)*dy * (double)(1<<16));
                r->weights[offset+3] = (int32_t)(dx*dy * (double)(1<<16));
            }

            matd_destroy(xy);
            matd_destroy(_ray);
            matd_destroy(ray);
            matd_destroy(ixy);
        }
    }

    return (rasterizer_t *)r;
}
