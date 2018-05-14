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
#include "common/zarray.h"

#include "camera_math.h"
#include "view.h"
#include "distortion_function_verifier.h"
#include "max_grown_inscribed_rectified_view.h"

#define VIEW_TYPE VIEW_MAX_GROWN_INSCRIBED_RECTIFIED_VIEW

struct _mgirv
{
    view_t * view;

    matd_t * xy01;

    matd_t * K;
    matd_t * Kinv;

    int width;
    int height;
};

// are all the points outside of the bounding box?
static int accept_move(zarray_t *border, double xmin, double xmax, double ymin, double ymax)
{
    int nborder = zarray_size(border);
    for (int i = 0; i < nborder; i++) {
        matd_t *xy = NULL;
        zarray_get(border, i, &xy);
        if (xy->data[0] > xmin && xy->data[0] < xmax && xy->data[1] > ymin && xy->data[1] < ymax)
            return 0;
    }

    return 1;
}

matd_t * mgirv_compute_xy01(const view_t *input, const matd_t * Krect)
{
    int input_width  = input->get_width(input);
    int input_height = input->get_height(input);

    dfv_t *dfv = dfv_create(input, 0.10, M_PI/1000);

    zarray_t *border = dfv_compute_rectified_border(dfv, input, Krect, 1.0);
    int nborder = zarray_size(border);

    dfv_destroy(dfv);

    double xmean = 0, ymean = 0;

    for (int i = 0; i < nborder; i++) {
        matd_t *xy_rp = NULL;
        zarray_get(border, i, &xy_rp);
        xmean += xy_rp->data[0]/nborder;
        ymean += xy_rp->data[1]/nborder;
    }

    int xmin = (int) xmean;
    int xmax = (int) xmean;
    int ymin = (int) ymean;
    int ymax = (int) ymean;

    int changed = 1;

    while (changed)
    {
        changed = 0;

        if (accept_move(border, xmin-1, xmax, ymin, ymax)) {
            changed = 1;
            xmin--;
        }

        if (accept_move(border, xmin, xmax+1, ymin, ymax)) {
            changed = 1;
            xmax++;
        }

        if (accept_move(border, xmin, xmax, ymin-1, ymax)) {
            changed = 1;
            ymin--;
        }

        if (accept_move(border, xmin, xmax, ymin, ymax+1)) {
            changed = 1;
            ymax++;
        }

        // crude check for a run-away rectified view
        double area = (xmax-xmin)*(ymax-ymin);
        if (area > 10*input_width*input_height) // XXX
            break;
    }

    zarray_vmap(border, matd_destroy);
    zarray_destroy(border);

    matd_t * xy01 = matd_create(2,2);

    xy01->data[0*xy01->ncols+0] = xmin;
    xy01->data[1*xy01->ncols+0] = xmax;
    xy01->data[0*xy01->ncols+1] = ymin;
    xy01->data[1*xy01->ncols+1] = ymax;

    return xy01;
}

mgirv_t * mgirv_create(const view_t *input)
{
    view_t  *view  = calloc(1, sizeof(view_t));
    mgirv_t *mgirv = calloc(1, sizeof(mgirv_t));

    // VIEW
    view->impl_type       = VIEW_TYPE;
    view->impl            = (void*) mgirv;
    view->get_width       = mgirv_get_width;
    view->get_height      = mgirv_get_height;
    view->copy_intrinsics = mgirv_copy_intrinsics;
    view->ray_to_pixels   = mgirv_ray_to_pixels;
    view->pixels_to_ray   = mgirv_pixels_to_ray;
    view->destroy         = mgirv_destroy_view;

    // MAX RECTIFIED VIEW
    mgirv->view = view;

    matd_t * K = input->copy_intrinsics(input);

    mgirv->xy01 = mgirv_compute_xy01(input, K);
    double xmin = mgirv->xy01->data[0*mgirv->xy01->ncols + 0];
    double xmax = mgirv->xy01->data[1*mgirv->xy01->ncols + 0];
    double ymin = mgirv->xy01->data[0*mgirv->xy01->ncols + 1];
    double ymax = mgirv->xy01->data[1*mgirv->xy01->ncols + 1];

    ////////////////////////////////////////
    // update K
    K->data[0*K->ncols + 2] -= xmin;
    K->data[1*K->ncols + 2] -= ymin;

    mgirv->K    = K;
    mgirv->Kinv = matd_inverse(mgirv->K);

    mgirv->width  = (int) floorf(xmax - xmin + 1);
    mgirv->height = (int) floorf(ymax - ymin + 1);

    printf("mgirv: xmin %12.6f xmax %12.6f ymin %12.6f ymax %12.6f width %d height %d\n",
           xmin, xmax, ymin, ymax, mgirv->width, mgirv->height);

    matd_print(mgirv->K, "%12.6f");

    mgirv->width  &= 0xFFFFFFFC;

    return mgirv;
}

view_t * mgirv_get_view(mgirv_t *mgirv)
{
    return mgirv->view;
}

int mgirv_get_width(const view_t *view)
{
    assert(view->impl_type == VIEW_TYPE);
    const mgirv_t *mgirv = (mgirv_t*) view->impl;

    return mgirv->width;
}

int mgirv_get_height(const view_t *view)
{
    assert(view->impl_type == VIEW_TYPE);
    const mgirv_t *mgirv = (mgirv_t*) view->impl;

    return mgirv->height;
}

matd_t * mgirv_copy_intrinsics(const view_t *view)
{
    assert(view->impl_type == VIEW_TYPE);
    const mgirv_t *mgirv = (mgirv_t*) view->impl;

    return matd_copy(mgirv->K);
}

matd_t * mgirv_ray_to_pixels(const view_t *view, const matd_t * xyz_r)
{
    assert(view->impl_type == VIEW_TYPE);
    const mgirv_t *mgirv = (mgirv_t*) view->impl;

    return pinhole_transform(mgirv->K, xyz_r);
}

matd_t * mgirv_pixels_to_ray(const view_t *view, const matd_t * xy_p)
{
    assert(view->impl_type == VIEW_TYPE);
    const mgirv_t *mgirv = (mgirv_t*) view->impl;

    matd_t *xy_rn = pinhole_transform(mgirv->Kinv, xy_p);
    matd_t *xyz_r = snap_ray_to_plane(xy_rn);
    matd_destroy(xy_rn);

    return xyz_r;
}

void mgirv_destroy_view(view_t *view)
{
    assert(view->impl_type == VIEW_TYPE);
    mgirv_t *mgirv = (mgirv_t*) view->impl;

    mgirv_destroy(mgirv);
}

void mgirv_destroy(mgirv_t *mgirv)
{
    free(mgirv->view);

    matd_destroy(mgirv->xy01);
    matd_destroy(mgirv->K);
    matd_destroy(mgirv->Kinv);

    free(mgirv);
}

