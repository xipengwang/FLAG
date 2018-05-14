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
#include <assert.h>

#include "camera_math.h"
#include "view.h"

#include "stereo_rectified_view.h"

#define VIEW_TYPE VIEW_STEREO_RECTIFIED_VIEW



srv_t *  srv_create(const matd_t *K, const matd_t *xy01)
{
    view_t *view = calloc(1, sizeof(view_t));
    srv_t  *srv  = calloc(1, sizeof(srv_t));

    // VIEW
    view->impl_type       = VIEW_TYPE;
    view->impl            = (void*) srv;
    view->get_width       = srv_get_width;
    view->get_height      = srv_get_height;
    view->copy_intrinsics = srv_copy_intrinsics;
    view->ray_to_pixels   = srv_ray_to_pixels;
    view->pixels_to_ray   = srv_pixels_to_ray;
    view->destroy         = srv_destroy_view;

    // STEREO RECTIFIED VIEW
    srv->view = view;
    srv->xy01 = matd_copy(xy01);

    double xmin = xy01->data[0*xy01->ncols + 0];
    double xmax = xy01->data[1*xy01->ncols + 0];
    double ymin = xy01->data[0*xy01->ncols + 1];
    double ymax = xy01->data[1*xy01->ncols + 1];

    srv->K = matd_copy(K);
    srv->K->data[0*srv->K->ncols+2] -= xmin;
    srv->K->data[1*srv->K->ncols+2] -= ymin;
    srv->Kinv = matd_inverse(srv->K);

    srv->width  = (int) floor(xmax - xmin + 1);
    srv->height = (int) floor(ymax - ymin + 1);

    srv->width &= 0xFFFFFFFC;

    return srv;
}

view_t * srv_get_view(srv_t *srv)
{
    return srv->view;
}

int srv_get_width(const view_t *view)
{
    assert(view->impl_type == VIEW_TYPE);
    srv_t *srv = (srv_t*) view->impl;

    return srv->width;
}

int srv_get_height(const view_t *view)
{
    assert(view->impl_type == VIEW_TYPE);
    srv_t *srv = (srv_t*) view->impl;

    return srv->height;
}

matd_t * srv_copy_intrinsics(const view_t *view)
{
    assert(view->impl_type == VIEW_TYPE);
    srv_t *srv = (srv_t*) view->impl;

    return matd_copy(srv->K);
}

matd_t * srv_ray_to_pixels(const view_t *view, const matd_t *xyz_r)
{
    assert(view->impl_type == VIEW_TYPE);
    srv_t *srv = (srv_t*) view->impl;

    return pinhole_transform(srv->K, xyz_r);
}

matd_t * srv_pixels_to_ray(const view_t *view, const matd_t *xy_rp)
{
    assert(view->impl_type == VIEW_TYPE);
    srv_t *srv = (srv_t*) view->impl;

    matd_t *xy_rn = pinhole_transform(srv->Kinv, xy_rp);
    matd_t *xyz_r = snap_ray_to_plane(xy_rn);
    matd_destroy(xy_rn);

    return xyz_r;
}

void srv_destroy_view(view_t *view)
{
    assert(view->impl_type == VIEW_TYPE);
    srv_t *srv = (srv_t*) view->impl;

    srv_destroy(srv);
}

void srv_destroy(srv_t *srv)
{
    free(srv->view);

    matd_destroy(srv->xy01);
    matd_destroy(srv->K);
    matd_destroy(srv->Kinv);

    free(srv);
}
