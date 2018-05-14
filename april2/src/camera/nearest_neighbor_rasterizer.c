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
#include <stdlib.h>

#include "common/image_u8.h"
#include "common/matd.h"

#include "camera_math.h"
#include "view.h"
#include "rasterizer.h"
#include "distortion_function_verifier.h"
#include "nearest_neighbor_rasterizer.h"

typedef struct
{
    int16_t x;
    int16_t y;
} index_t;

typedef struct _impl impl_t;
struct _impl
{
    int input_width;
    int input_height;

    int output_width;
    int output_height;

    uint32_t nindices;
    index_t  *indices; // -1 indicates an invalid mapping
};

rasterizer_t * nearest_neighbor_rasterizer_create(const view_t *input, const view_t *output)
{
    return nearest_neighbor_rasterizer_create_rot(input, NULL, output, NULL);
}

rasterizer_t * nearest_neighbor_rasterizer_create_rot(const view_t *input,  const matd_t * G2C_input,
                                                      const view_t *output, const matd_t * G2C_output)
{
    rasterizer_t *rasterizer = calloc(1, sizeof(rasterizer_t));
    impl_t       *impl       = calloc(1, sizeof(impl_t));

    rasterizer->impl_type     = RASTERIZER_NEAREST_NEIGHBOR;
    rasterizer->impl          = (void*) impl;
    rasterizer->rectify_image_u8   = nearest_neighbor_rasterizer_rectify_image_u8;
    rasterizer->rectify_image_u8x3 = nearest_neighbor_rasterizer_rectify_image_u8x3;
    rasterizer->rectify_image_u8x4 = nearest_neighbor_rasterizer_rectify_image_u8x4;
    rasterizer->destroy       = nearest_neighbor_rasterizer_destroy;

    dfv_t *in_verifier  = dfv_create(input,  0.10, M_PI/1000);
    dfv_t *out_verifier = dfv_create(output, 0.10, M_PI/1000);

    ////////////////////////////////////////
    impl->input_width  = input->get_width(input);
    impl->input_height = input->get_height(input);

    impl->output_width  = output->get_width(output);
    impl->output_height = output->get_height(output);

    impl->nindices = impl->output_width * impl->output_height;
    impl->indices = malloc(impl->nindices * sizeof(index_t));

    for (int i = 0; i < impl->nindices; i++) {
        impl->indices[i].x = -1;
        impl->indices[i].y = -1;
    }

    ////////////////////////////////////////
    // optional rotation to convert from output orientation to input orientation
    matd_t * R_OutToIn = NULL;
    if (G2C_input != NULL && G2C_output != NULL) {
        matd_t *R_input  = matd_select(G2C_input,  0, 2, 0, 2);
        matd_t *R_output = matd_select(G2C_output, 0, 2, 0, 2);

        R_OutToIn = matd_op("MM^-1", R_input, R_output);
        assert(R_OutToIn != NULL);

        matd_destroy(R_input);
        matd_destroy(R_output);
    }

    ////////////////////////////////////////
    // build table

    for (int y_rp = 0; y_rp < impl->output_height; y_rp++) {
        for (int x_rp = 0; x_rp < impl->output_width; x_rp++) {

            matd_t *xy_rp = matd_create(2, 1);
            xy_rp->data[0] = x_rp;
            xy_rp->data[1] = y_rp;

            if (!dfv_pixel_valid(out_verifier, xy_rp)) {
                matd_destroy(xy_rp);
                continue;
            }

            matd_t *xyz_r_raw   = output->pixels_to_ray(output, xy_rp);
            matd_t *xyz_r_plane = snap_ray_to_plane(xyz_r_raw);
            matd_t *xyz_r_rot   = NULL;

            if (R_OutToIn == NULL) xyz_r_rot = matd_copy(xyz_r_plane);
            else                   xyz_r_rot = point_transform(R_OutToIn, xyz_r_plane);

            matd_destroy(xy_rp);
            matd_destroy(xyz_r_raw);
            matd_destroy(xyz_r_plane);

            if (!dfv_ray_valid(in_verifier, xyz_r_rot)) {
                matd_destroy(xyz_r_rot);
                continue;
            }

            matd_t * xy_dp = input->ray_to_pixels(input, xyz_r_rot);
            matd_destroy(xyz_r_rot);

            // Our convention is that a pixel ranges from -0.5/+0.5 around its index
            // (e.g. x=0 ranges from x=-0.5/+0.5). For nearest neighbor lookup, we
            // round to simply snap to the nearest pixel
            int32_t x_dp = (int32_t) round(xy_dp->data[0]);
            int32_t y_dp = (int32_t) round(xy_dp->data[1]);

            matd_destroy(xy_dp);

            if (x_dp < 0 || y_dp < 0 || x_dp+1 >= impl->input_width || y_dp+1 >= impl->input_height)
                continue;

            impl->indices[y_rp*impl->output_width + x_rp].x = x_dp;
            impl->indices[y_rp*impl->output_width + x_rp].y = y_dp;
        }
    }

    dfv_destroy(in_verifier);
    dfv_destroy(out_verifier);

    return rasterizer;
}

void nearest_neighbor_rasterizer_destroy(rasterizer_t *rasterizer)
{
    assert(rasterizer->impl_type == RASTERIZER_NEAREST_NEIGHBOR);
    impl_t *impl = (impl_t*) rasterizer->impl;

    free(impl->indices);
    free(impl);

    free(rasterizer);
}

image_u8_t *nearest_neighbor_rasterizer_rectify_image_u8(const rasterizer_t *rasterizer,
                                                         const image_u8_t *inp)
{
    assert(rasterizer->impl_type == RASTERIZER_NEAREST_NEIGHBOR);
    impl_t *impl = (impl_t*) rasterizer->impl;
    index_t *indices = impl->indices;

    assert(inp->width  == impl->input_width);
    assert(inp->height == impl->input_height);
    const image_u8_t in = *inp;

    image_u8_t *outp = image_u8_create(impl->output_width, impl->output_height);
    image_u8_t out = *outp;

    for (int y_rp = 0; y_rp < out.height; y_rp++) {
        for (int x_rp = 0; x_rp < out.width; x_rp++) {

            int i = y_rp*out.width + x_rp; // index when stride was unknown

            index_t index = indices[i];
            if (index.x == -1 || index.y == -1)
                continue;

            int idx = index.y*in.stride + index.x;

            out.buf[y_rp*out.stride + x_rp] = in.buf[idx];
        }
    }

    return outp;
}

image_u8x3_t *nearest_neighbor_rasterizer_rectify_image_u8x3(const rasterizer_t *rasterizer,
                                                             const image_u8x3_t *inp)
{
    assert(rasterizer->impl_type == RASTERIZER_NEAREST_NEIGHBOR);
    impl_t *impl = (impl_t*) rasterizer->impl;
    index_t *indices = impl->indices;

    assert(inp->width  == impl->input_width);
    assert(inp->height == impl->input_height);
    const image_u8x3_t in = *inp;

    image_u8x3_t *outp = image_u8x3_create(impl->output_width, impl->output_height);
    image_u8x3_t out = *outp;

    for (int y_rp = 0; y_rp < out.height; y_rp++) {
        for (int x_rp = 0; x_rp < out.width; x_rp++) {

            int i = y_rp*out.width + x_rp; // index when stride was unknown

            index_t index = indices[i];
            if (index.x == -1 || index.y == -1)
                continue;

            out.buf[y_rp*out.stride + 3*x_rp + 0] = in.buf[index.y*in.stride + 3*index.x + 0];
            out.buf[y_rp*out.stride + 3*x_rp + 1] = in.buf[index.y*in.stride + 3*index.x + 1];
            out.buf[y_rp*out.stride + 3*x_rp + 2] = in.buf[index.y*in.stride + 3*index.x + 2];
        }
    }

    return outp;
}

image_u8x4_t *nearest_neighbor_rasterizer_rectify_image_u8x4(const rasterizer_t *rasterizer,
                                                             const image_u8x4_t *inp)
{
    assert(rasterizer->impl_type == RASTERIZER_NEAREST_NEIGHBOR);
    impl_t *impl = (impl_t*) rasterizer->impl;
    index_t *indices = impl->indices;

    assert(inp->width  == impl->input_width);
    assert(inp->height == impl->input_height);
    const image_u8x4_t in = *inp;

    image_u8x4_t *outp = image_u8x4_create(impl->output_width, impl->output_height);
    image_u8x4_t out = *outp;

    for (int y_rp = 0; y_rp < out.height; y_rp++) {
        for (int x_rp = 0; x_rp < out.width; x_rp++) {

            int i = y_rp*out.width + x_rp; // index when stride was unknown

            index_t index = indices[i];
            if (index.x == -1 || index.y == -1)
                continue;

            out.buf[y_rp*out.stride + 4*x_rp + 0] = in.buf[index.y*in.stride + 4*index.x + 0];
            out.buf[y_rp*out.stride + 4*x_rp + 1] = in.buf[index.y*in.stride + 4*index.x + 1];
            out.buf[y_rp*out.stride + 4*x_rp + 2] = in.buf[index.y*in.stride + 4*index.x + 2];
            out.buf[y_rp*out.stride + 4*x_rp + 3] = in.buf[index.y*in.stride + 4*index.x + 3];
        }
    }

    return outp;
}

