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
#include "max_rectified_view.h"

#define VIEW_TYPE VIEW_MAX_RECTIFIED_VIEW

struct _mrv
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

matd_t * mrv_compute_xy01(const view_t * input, const matd_t * Krect)
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

        xmin = fmin(xmin, xy_rp->data[0]);
        xmax = fmax(xmax, xy_rp->data[0]);

        ymin = fmin(ymin, xy_rp->data[1]);
        ymax = fmax(ymax, xy_rp->data[1]);

        matd_destroy(xy_rp);
    }

    // TR -> BR
    x_dp = input_width-1;
    for (y_dp = 0; y_dp < input_height; y_dp++) {

        matd_t *xy_rp = convert(input, verifier, Krect, x_dp, y_dp);

        xmin = fmin(xmin, xy_rp->data[0]);
        xmax = fmax(xmax, xy_rp->data[0]);

        ymin = fmin(ymin, xy_rp->data[1]);
        ymax = fmax(ymax, xy_rp->data[1]);

        matd_destroy(xy_rp);
    }

    // BR -> BL
    y_dp = input_height-1;
    for (x_dp = input_width-1; x_dp >= 0; x_dp--) {

        matd_t *xy_rp = convert(input, verifier, Krect, x_dp, y_dp);

        xmin = fmin(xmin, xy_rp->data[0]);
        xmax = fmax(xmax, xy_rp->data[0]);

        ymin = fmin(ymin, xy_rp->data[1]);
        ymax = fmax(ymax, xy_rp->data[1]);

        matd_destroy(xy_rp);
    }

    // BL -> TL
    x_dp = 0;
    for (y_dp = input_height-1; y_dp >= 0; y_dp--) {

        matd_t *xy_rp = convert(input, verifier, Krect, x_dp, y_dp);

        xmin = fmin(xmin, xy_rp->data[0]);
        xmax = fmax(xmax, xy_rp->data[0]);

        ymin = fmin(ymin, xy_rp->data[1]);
        ymax = fmax(ymax, xy_rp->data[1]);

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

mrv_t * mrv_create(const view_t *input)
{
    view_t *view = calloc(1, sizeof(view_t));
    mrv_t  *mrv  = calloc(1, sizeof(mrv_t));

    // VIEW
    view->impl_type       = VIEW_TYPE;
    view->impl            = (void*) mrv;
    view->get_width       = mrv_get_width;
    view->get_height      = mrv_get_height;
    view->copy_intrinsics = mrv_copy_intrinsics;
    view->ray_to_pixels   = mrv_ray_to_pixels;
    view->pixels_to_ray   = mrv_pixels_to_ray;
    view->destroy         = mrv_destroy_view;

    // MAX RECTIFIED VIEW
    mrv->view = view;

    matd_t * K = input->copy_intrinsics(input);

    mrv->xy01 = mrv_compute_xy01(input, K);
    double xmin = mrv->xy01->data[0*mrv->xy01->ncols + 0];
    double xmax = mrv->xy01->data[1*mrv->xy01->ncols + 0];
    double ymin = mrv->xy01->data[0*mrv->xy01->ncols + 1];
    double ymax = mrv->xy01->data[1*mrv->xy01->ncols + 1];

    // update K
    K->data[0*K->ncols + 2] -= xmin;
    K->data[1*K->ncols + 2] -= ymin;

    mrv->K    = K;
    mrv->Kinv = matd_inverse(mrv->K);

    mrv->width  = (int) floorf(xmax - xmin + 1);
    mrv->height = (int) floorf(ymax - ymin + 1);

    printf("mrv: xmin %12.6f xmax %12.6f ymin %12.6f ymax %12.6f width %d height %d\n",
           xmin, xmax, ymin, ymax, mrv->width, mrv->height);

    matd_print(mrv->K, "%12.6f");

    mrv->width  &= 0xFFFFFFFC;

    return mrv;
}

view_t * mrv_get_view(mrv_t *mrv)
{
    return mrv->view;
}

int mrv_get_width(const view_t *view)
{
    assert(view->impl_type == VIEW_TYPE);
    const mrv_t *mrv = (mrv_t*) view->impl;

    return mrv->width;
}

int mrv_get_height(const view_t *view)
{
    assert(view->impl_type == VIEW_TYPE);
    const mrv_t *mrv = (mrv_t*) view->impl;

    return mrv->height;
}

matd_t * mrv_copy_intrinsics(const view_t *view)
{
    assert(view->impl_type == VIEW_TYPE);
    const mrv_t *mrv = (mrv_t*) view->impl;

    return matd_copy(mrv->K);
}

matd_t * mrv_ray_to_pixels(const view_t *view, const matd_t * xyz_r)
{
    assert(view->impl_type == VIEW_TYPE);
    const mrv_t *mrv = (mrv_t*) view->impl;

    return pinhole_transform(mrv->K, xyz_r);
}

matd_t * mrv_pixels_to_ray(const view_t *view, const matd_t * xy_p)
{
    assert(view->impl_type == VIEW_TYPE);
    const mrv_t *mrv = (mrv_t*) view->impl;

    matd_t *xy_rn = pinhole_transform(mrv->Kinv, xy_p);
    matd_t *xyz_r = snap_ray_to_plane(xy_rn);
    matd_destroy(xy_rn);

    return xyz_r;
}

void mrv_destroy_view(view_t *view)
{
    assert(view->impl_type == VIEW_TYPE);
    mrv_t *mrv = (mrv_t*) view->impl;

    mrv_destroy(mrv);
}

void mrv_destroy(mrv_t *mrv)
{
    free(mrv->view);

    matd_destroy(mrv->xy01);
    matd_destroy(mrv->K);
    matd_destroy(mrv->Kinv);

    free(mrv);
}

