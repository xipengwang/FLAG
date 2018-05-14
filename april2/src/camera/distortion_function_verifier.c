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

#include "common/matd.h"
#include "common/zarray.h"

#include "camera_math.h"
#include "view.h"
#include "distortion_function_verifier.h"

struct _dfv
{
    double radius_buffer;
    double dtheta;

    double cx;
    double cy;

    double max_valid_theta; // angle off of z axis
    double max_valid_pixel_radius;
};

double dist(double a[2], double b[2])
{
    double dx = a[0] - b[0];
    double dy = a[1] - b[1];

    return sqrt(dx*dx + dy*dy);
}

double get_max_pixel_radius(double cx, double cy, int width, int height)
{
    double max = 0;

    double cc[] = { cx, cy };

    double c00[] = { 0,       0        };
    double c10[] = { width-1, 0        };
    double c01[] = { 0,       height-1 };
    double c11[] = { width-1, height-1 };

    max = fmax(max, dist(cc, c00));
    max = fmax(max, dist(cc, c10));
    max = fmax(max, dist(cc, c01));
    max = fmax(max, dist(cc, c11));

    return max;
}

dfv_t * dfv_create(const view_t *view, double radius_buffer, double dtheta)
{
    dfv_t *dfv          = calloc(1, sizeof(dfv_t));
    dfv->radius_buffer  = radius_buffer;
    dfv->dtheta         = dtheta;

    matd_t *K = view->copy_intrinsics(view);
    dfv->cx = K->data[0*3 + 2];
    dfv->cy = K->data[1*3 + 2];
    matd_destroy(K);

    int width = view->get_width(view);
    int height = view->get_height(view);

    double max_pixel_radius = get_max_pixel_radius(dfv->cx, dfv->cy, width, height);

    double max_theta = M_PI;
    double last_pixel_radius = 0;

    for (double theta = 0; theta < max_theta; theta += dfv->dtheta)
    {
        double x = sin(theta); // if y==0; x==r. sin(theta) = O/H = r/1 = r = x
        double y = 0;
        double z = cos(theta); // cos(theta) = A/H = z/1 = z

        matd_t *xyz_r = matd_create(3, 1);
        xyz_r->data[0] = x;
        xyz_r->data[1] = y;
        xyz_r->data[2] = z;

        matd_t *xy_dp = view->ray_to_pixels(view, xyz_r);

        double dx = xy_dp->data[0] - dfv->cx;
        double dy = xy_dp->data[1] - dfv->cy;
        double pixel_radius = sqrt(dx*dx + dy*dy);

        matd_destroy(xyz_r);
        matd_destroy(xy_dp);

        if (pixel_radius < last_pixel_radius)
            break;

        dfv->max_valid_theta = theta;
        dfv->max_valid_pixel_radius = pixel_radius;

        last_pixel_radius = pixel_radius;

        // break if we're past the furthest corner in the distorted image.
        // we add a user-configurable buffer because projections just outside
        // of the image can be useful
        if (pixel_radius > (1.0 + dfv->radius_buffer)*max_pixel_radius)
            break;
    }

    return dfv;
}

void dfv_destroy(dfv_t *dfv)
{
    free(dfv);
}

int dfv_ray_valid(const dfv_t *dfv, const matd_t *xyz_r)
{
    assert(xyz_r->nrows*xyz_r->ncols == 3);

    double x = xyz_r->data[0];
    double y = xyz_r->data[1];
    double z = xyz_r->data[2];

    double r     = sqrt(x*x + y*y);
    double theta = atan2(r, z);

    if (theta < dfv->max_valid_theta)
        return 1;

    return 0;
}

int dfv_pixel_valid(const dfv_t *dfv, const matd_t *xy_dp)
{
    assert(xy_dp->nrows*xy_dp->ncols == 2);

    double dx = xy_dp->data[0] - dfv->cx;
    double dy = xy_dp->data[1] - dfv->cy;
    double pixel_radius = sqrt(dx*dx + dy*dy);

    if (pixel_radius < dfv->max_valid_pixel_radius)
        return 1;

    return 0;
}

matd_t * dfv_clamp_ray(const dfv_t *dfv, const matd_t *xyz_r)
{
    assert(xyz_r->nrows*xyz_r->ncols == 3);

    double x = xyz_r->data[0];
    double y = xyz_r->data[1];
    double z = xyz_r->data[2];

    double r     = sqrt(x*x + y*y);
    double theta = atan2(r, z);
    double psi   = atan2(y, x);

    if (theta < dfv->max_valid_theta)
        return matd_copy(xyz_r);

    theta = dfv->max_valid_theta;

    z = cos(theta); // cos(theta) = A/H = z/1 = z
    r = sin(theta); // sin(theta) = O/H = r/1 = r

    x = r*cos(psi);
    y = r*sin(psi);

    matd_t *clamped = matd_create(3, 1);
    clamped->data[0] = x;
    clamped->data[1] = y;
    clamped->data[2] = z;

    return clamped;
}

matd_t * dfv_clamp_pixels(const dfv_t *dfv, const matd_t *xy_dp)
{
    assert(xy_dp->nrows*xy_dp->ncols == 2);

    double dx = xy_dp->data[0] - dfv->cx;
    double dy = xy_dp->data[1] - dfv->cy;
    double pixel_radius = sqrt(dx*dx + dy*dy);

    if (pixel_radius < dfv->max_valid_pixel_radius)
        return matd_copy(xy_dp);

    matd_t *clamped = matd_create(2, 1);
    clamped->data[0] = dfv->cx + dx*dfv->max_valid_pixel_radius/pixel_radius;
    clamped->data[1] = dfv->cy + dy*dfv->max_valid_pixel_radius/pixel_radius;

    return clamped;
}

// "private"
static matd_t * rectify_pixel(const view_t * view, const dfv_t * dfv, const matd_t * K, double x_dp, double y_dp)
{
    matd_t *xy_dp  = matd_create(2, 1);
    xy_dp->data[0] = x_dp;
    xy_dp->data[1] = y_dp;

    matd_t *xy_dp_clamped = dfv_clamp_pixels(dfv, xy_dp);
    matd_t *xyz_r         = view->pixels_to_ray(view, xy_dp_clamped);
    matd_t *xy_rp         = pinhole_transform(K, xyz_r);

    matd_destroy(xy_dp);
    matd_destroy(xy_dp_clamped);
    matd_destroy(xyz_r);

    return xy_rp;
}

zarray_t * dfv_compute_rectified_border(const dfv_t *dfv, const view_t * view, const matd_t * K, double stepsize)
{
    zarray_t * border = zarray_create(sizeof(matd_t*));

    int width  = view->get_width(view);
    int height = view->get_height(view);

    int x_dp, y_dp;

    x_dp = 0;
    for (y_dp = 0; y_dp < height; y_dp += stepsize) {
        matd_t *xy_rp = rectify_pixel(view, dfv, K, x_dp, y_dp);
        zarray_add(border, &xy_rp);
    }

    y_dp = height-1;
    for (x_dp = 0; x_dp < width; x_dp += stepsize) {
        matd_t *xy_rp = rectify_pixel(view, dfv, K, x_dp, y_dp);
        zarray_add(border, &xy_rp);
    }

    x_dp = width-1;
    for (y_dp = height-1; y_dp >= 0; y_dp -= stepsize) {
        matd_t *xy_rp = rectify_pixel(view, dfv, K, x_dp, y_dp);
        zarray_add(border, &xy_rp);
    }

    y_dp = 0;
    for (x_dp = width-1; x_dp >= 0; x_dp -= stepsize) {
        matd_t *xy_rp = rectify_pixel(view, dfv, K, x_dp, y_dp);
        zarray_add(border, &xy_rp);
    }

    return border;
}
