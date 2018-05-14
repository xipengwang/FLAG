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
#include "scaled_view.h"

#define VIEW_TYPE VIEW_SCALED_VIEW

struct _sv
{
    view_t * view;
    const view_t * input;

    double scale;

    matd_t * S;
    matd_t * Sinv;

    int width;
    int height;
};

sv_t * sv_create(double scale, const view_t *input)
{
    view_t *view = calloc(1, sizeof(view_t));
    sv_t   *sv   = calloc(1, sizeof(sv_t));

    // VIEW
    view->impl_type       = VIEW_TYPE;
    view->impl            = (void*) sv;
    view->get_width       = sv_get_width;
    view->get_height      = sv_get_height;
    view->copy_intrinsics = sv_copy_intrinsics;
    view->ray_to_pixels   = sv_ray_to_pixels;
    view->pixels_to_ray   = sv_pixels_to_ray;
    view->destroy         = sv_destroy_view;

    // SCALED VIEW
    sv->view  = view;
    sv->input = input;

    double offs = (1.0 - scale) / 2;

    double Sarray[] = { scale,     0, -offs,
                            0, scale, -offs,
                            0,     0,     1 };

    sv->S = matd_create_data(3,3,Sarray);
    sv->Sinv = matd_inverse(sv->S);

    sv->width  = (int) floorf(input->get_width(input)  * scale);
    sv->height = (int) floorf(input->get_height(input) * scale);

    sv->width  &= 0xFFFFFFFC;

    return sv;
}

view_t * sv_get_view(sv_t *sv)
{
    return sv->view;
}

int sv_get_width(const view_t *view)
{
    assert(view->impl_type == VIEW_TYPE);
    const sv_t *sv = (sv_t*) view->impl;

    return sv->width;
}

int sv_get_height(const view_t *view)
{
    assert(view->impl_type == VIEW_TYPE);
    const sv_t *sv = (sv_t*) view->impl;

    return sv->height;
}

matd_t * sv_copy_intrinsics(const view_t *view)
{
    assert(view->impl_type == VIEW_TYPE);
    const sv_t *sv = (sv_t*) view->impl;

    matd_t *K = sv->input->copy_intrinsics(sv->input);

    matd_t *SK = matd_op("MM", sv->S, K);

    matd_destroy(K);

    return SK;
}

matd_t * sv_ray_to_pixels(const view_t *view, const matd_t * xyz_r)
{
    assert(view->impl_type == VIEW_TYPE);
    const sv_t *sv = (sv_t*) view->impl;

    matd_t *xy_p  = sv->input->ray_to_pixels(sv->input, xyz_r);
    matd_t *xy_ps = pinhole_transform(sv->S, xy_p);
    matd_destroy(xy_p);
    return xy_ps;
}

matd_t * sv_pixels_to_ray(const view_t *view, const matd_t * xy_ps)
{
    assert(view->impl_type == VIEW_TYPE);
    const sv_t *sv = (sv_t*) view->impl;

    matd_t *xy_p = pinhole_transform(sv->Sinv, xy_ps);

    matd_t *xyz_r = sv->input->pixels_to_ray(sv->input, xy_p);

    matd_destroy(xy_p);
    return xyz_r;
}

void sv_destroy_view(view_t *view)
{
    assert(view->impl_type == VIEW_TYPE);
    sv_t *sv = (sv_t*) view->impl;

    sv_destroy(sv);
}

void sv_destroy(sv_t *sv)
{
    free(sv->view);

    matd_destroy(sv->S);
    matd_destroy(sv->Sinv);

    free(sv);
}

