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

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#include "common/matd.h"

#include "camera_math.h"
#include "view.h"
#include "distortion_function_verifier.h"
#include "max_inscribed_rectified_view.h"

#define VIEW_TYPE VIEW_MAX_INSCRIBED_RECTIFIED_VIEW

#define DEBUG 0

struct _mirv
{
    view_t * view;

    matd_t * xy01;

    matd_t * K;
    matd_t * Kinv;

    int width;
    int height;
};

static matd_t * convert(const view_t *input, const dfv_t *verifier, const matd_t *Krect, double x_dp, double y_dp)
{
    matd_t *xy_dp  = matd_create(2, 1);
    xy_dp->data[0] = x_dp;
    xy_dp->data[1] = y_dp;

    matd_t *xy_dp_clamped = dfv_clamp_pixels(verifier, xy_dp);
    matd_t *xyz_r         = input->pixels_to_ray(input, xy_dp_clamped);
    matd_t *xy_rp         = pinhole_transform(Krect, xyz_r);

    matd_destroy(xy_dp);
    matd_destroy(xy_dp_clamped);
    matd_destroy(xyz_r);

    return xy_rp;
}

matd_t * mirv_compute_xy01(const view_t *input, const matd_t * Krect)
{
    int x_dp, y_dp;

    int input_width  = input->get_width(input);
    int input_height = input->get_height(input);

    dfv_t *verifier = dfv_create(input, 0.10, M_PI/1000);

    double xmin, xmax, ymin, ymax;

    ////////////////////////////////////////
    // initialize border

    {
        matd_t *xy_rp = convert(input, verifier, Krect, 0, 0);
        xmin = xy_rp->data[0];
        ymin = xy_rp->data[1];
        matd_destroy(xy_rp);
    }

    {
        matd_t *xy_rp = convert(input, verifier, Krect, input_width-1, input_height-1);
        xmax = xy_rp->data[0];
        ymax = xy_rp->data[1];
        matd_destroy(xy_rp);
    }

    ////////////////////////////////////////
    // walk full border

    // TL -> TR
    y_dp = 0;
    for (x_dp = 0; x_dp < input_width; x_dp++) {

        matd_t *xy_rp = convert(input, verifier, Krect, x_dp, y_dp);
        ymin = fmax(ymin, xy_rp->data[1]);
        matd_destroy(xy_rp);
    }

    // TR -> BR
    x_dp = input_width-1;
    for (y_dp = 0; y_dp < input_height; y_dp++) {

        matd_t *xy_rp = convert(input, verifier, Krect, x_dp, y_dp);
        xmax = fmin(xmax, xy_rp->data[0]);
        matd_destroy(xy_rp);
    }

    // BR -> BL
    y_dp = input_height-1;
    for (x_dp = input_width-1; x_dp >= 0; x_dp--) {

        matd_t *xy_rp = convert(input, verifier, Krect, x_dp, y_dp);
        ymax = fmin(ymax, xy_rp->data[1]);
        matd_destroy(xy_rp);
    }

    // BL -> TL
    x_dp = 0;
    for (y_dp = input_height-1; y_dp >= 0; y_dp--) {

        matd_t *xy_rp = convert(input, verifier, Krect, x_dp, y_dp);
        xmin = fmax(xmin, xy_rp->data[0]);
        matd_destroy(xy_rp);
    }

    dfv_destroy(verifier);

    matd_t * xy01 = matd_create(2,2);
    xy01->data[0*xy01->ncols + 0] = xmin;
    xy01->data[1*xy01->ncols + 0] = xmax;
    xy01->data[0*xy01->ncols + 1] = ymin;
    xy01->data[1*xy01->ncols + 1] = ymax;

    return xy01;
}

mirv_t * mirv_create(const view_t *input)
{
    view_t *view = calloc(1, sizeof(view_t));
    mirv_t  *mirv  = calloc(1, sizeof(mirv_t));

    // VIEW
    view->impl_type       = VIEW_TYPE;
    view->impl            = (void*) mirv;
    view->get_width       = mirv_get_width;
    view->get_height      = mirv_get_height;
    view->copy_intrinsics = mirv_copy_intrinsics;
    view->ray_to_pixels   = mirv_ray_to_pixels;
    view->pixels_to_ray   = mirv_pixels_to_ray;
    view->destroy         = mirv_destroy_view;

    // MAX RECTIFIED VIEW
    mirv->view = view;

    matd_t *K = input->copy_intrinsics(input);

    mirv->xy01 = mirv_compute_xy01(input, K);
    double xmin = mirv->xy01->data[0*mirv->xy01->ncols + 0];
    double xmax = mirv->xy01->data[1*mirv->xy01->ncols + 0];
    double ymin = mirv->xy01->data[0*mirv->xy01->ncols + 1];
    double ymax = mirv->xy01->data[1*mirv->xy01->ncols + 1];

    // update K
    K->data[0*K->ncols + 2] -= xmin;
    K->data[1*K->ncols + 2] -= ymin;

    mirv->K    = K;
    mirv->Kinv = matd_inverse(mirv->K);

    mirv->width  = (int) floorf(xmax - xmin + 1);
    mirv->height = (int) floorf(ymax - ymin + 1);

#if DEBUG
    printf("mirv: xmin %12.6f xmax %12.6f ymin %12.6f ymax %12.6f width %d height %d\n",
           xmin, xmax, ymin, ymax, mirv->width, mirv->height);

    matd_print(mirv->K, "%12.6f");
#endif

    mirv->width  &= 0xFFFFFFFC;

    return mirv;
}

view_t * mirv_get_view(mirv_t *mirv)
{
    return mirv->view;
}

matd_t * mirv_get_bounds_xy01(const mirv_t * mirv)
{
    return matd_copy(mirv->xy01);
}

int mirv_get_width(const view_t *view)
{
    assert(view->impl_type == VIEW_TYPE);
    const mirv_t *mirv = (mirv_t*) view->impl;

    return mirv->width;
}

int mirv_get_height(const view_t *view)
{
    assert(view->impl_type == VIEW_TYPE);
    const mirv_t *mirv = (mirv_t*) view->impl;

    return mirv->height;
}

matd_t * mirv_copy_intrinsics(const view_t *view)
{
    assert(view->impl_type == VIEW_TYPE);
    const mirv_t *mirv = (mirv_t*) view->impl;

    return matd_copy(mirv->K);
}

matd_t * mirv_ray_to_pixels(const view_t *view, const matd_t * xyz_r)
{
    assert(view->impl_type == VIEW_TYPE);
    const mirv_t *mirv = (mirv_t*) view->impl;

    return pinhole_transform(mirv->K, xyz_r);
}

matd_t * mirv_pixels_to_ray(const view_t *view, const matd_t * xy_p)
{
    assert(view->impl_type == VIEW_TYPE);
    const mirv_t *mirv = (mirv_t*) view->impl;

    matd_t *xy_rn = pinhole_transform(mirv->Kinv, xy_p);
    matd_t *xyz_r = snap_ray_to_plane(xy_rn);
    matd_destroy(xy_rn);

    return xyz_r;
}

void mirv_destroy_view(view_t *view)
{
    assert(view->impl_type == VIEW_TYPE);
    mirv_t *mirv = (mirv_t*) view->impl;

    mirv_destroy(mirv);
}

void mirv_destroy(mirv_t *mirv)
{
    free(mirv->view);

    matd_destroy(mirv->xy01);
    matd_destroy(mirv->K);
    matd_destroy(mirv->Kinv);

    free(mirv);
}

