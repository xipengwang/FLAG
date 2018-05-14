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
#include <stdlib.h>

#include "common/config.h"
#include "common/matd.h"
#include "common/string_util.h"

#include "camera_math.h"
#include "view.h"
#include "calibration.h"
#include "distortion_free_calibration.h"

// to make it easier to create new implementations
#define VIEW_TYPE VIEW_CALIBRATION
#define CALIBRATION_TYPE CALIBRATION_DISTORTION_FREE

struct _dfc
{
    view_t *view;
    calibration_t *cal;

    // model parameters
    int width;
    int height;

    matd_t *fc; // always 2x1
    matd_t *cc; // always 2x1

    matd_t *K;
    matd_t *Kinv;
};

dfc_t * dfc_create(const matd_t *fc, const matd_t * cc, int width, int height)
{
    assert(fc->nrows*fc->ncols == 2);
    assert(cc->nrows*cc->ncols == 2);

    view_t        *view = calloc(1, sizeof(view_t));
    calibration_t *cal  = calloc(1, sizeof(calibration_t));
    dfc_t         *dfc  = calloc(1, sizeof(dfc_t));

    // VIEW
    view->impl_type       = VIEW_TYPE;
    view->impl            = (void*) cal;
    view->get_width       = dfc_get_width;
    view->get_height      = dfc_get_height;
    view->copy_intrinsics = dfc_copy_intrinsics;
    view->ray_to_pixels   = dfc_ray_to_pixels;
    view->pixels_to_ray   = dfc_pixels_to_ray;
    view->destroy         = dfc_destroy_view;

    // CALIBRATION
    cal->view                   = view;
    cal->impl_type              = CALIBRATION_TYPE;
    cal->impl                   = (void*) dfc;
    cal->get_calibration_string = dfc_get_calibration_string;
    cal->destroy                = dfc_destroy_cal;

    // DISTORTION FREE CALIBRATION
    dfc->view = view;
    dfc->cal  = cal;

    dfc->width  = width;
    dfc->height = height;

    dfc->fc = matd_copy(fc);
    dfc->cc = matd_copy(cc);

    matd_t *K = matd_create(3, 3);
    K->data[0*3+0] = fc->data[0];
    K->data[1*3+1] = fc->data[1];
    K->data[0*3+2] = cc->data[0];
    K->data[1*3+2] = cc->data[1];
    K->data[2*3+2] = 1;

    dfc->K    = K;
    dfc->Kinv = matd_inverse(K);

    return dfc;
}

dfc_t *  dfc_config_create(config_t *config, const char* child)
{
    char* key;

    key = sprintf_alloc("%s.intrinsics.fc", child);
    const matd_t * fc = config_require_matd(config, key);
    free(key);

    key = sprintf_alloc("%s.intrinsics.cc", child);
    const matd_t * cc = config_require_matd(config, key);
    free(key);

    key = sprintf_alloc("%s.width", child);
    int width = config_require_int(config, key);
    free(key);

    key = sprintf_alloc("%s.height", child);
    int height = config_require_int(config, key);
    free(key);

    return dfc_create(fc, cc, width, height);
}

view_t * dfc_get_view(dfc_t *dfc)
{
    return dfc->view;
}

calibration_t * dfc_get_cal(dfc_t *dfc)
{
    return dfc->cal;
}

int dfc_get_width(const view_t *view)
{
    assert(view->impl_type == VIEW_TYPE);
    const calibration_t *cal = (calibration_t*) view->impl;

    assert(cal->impl_type == CALIBRATION_TYPE);
    const dfc_t *dfc = (dfc_t*) cal->impl;

    return dfc->width;
}

int dfc_get_height(const view_t *view)
{
    assert(view->impl_type == VIEW_TYPE);
    const calibration_t *cal = (calibration_t*) view->impl;

    assert(cal->impl_type == CALIBRATION_TYPE);
    const dfc_t *dfc = (dfc_t*) cal->impl;

    return dfc->height;
}

matd_t * dfc_copy_intrinsics(const view_t *view)
{
    assert(view->impl_type == VIEW_TYPE);
    const calibration_t *cal = (calibration_t*) view->impl;

    assert(cal->impl_type == CALIBRATION_TYPE);
    const dfc_t *dfc = (dfc_t*) cal->impl;

    return matd_copy(dfc->K);
}

matd_t * dfc_ray_to_pixels(const view_t *view, const matd_t * xyz_r)
{
    assert(view->impl_type == VIEW_TYPE);
    const calibration_t *cal = (calibration_t*) view->impl;

    assert(cal->impl_type == CALIBRATION_TYPE);
    const dfc_t *dfc = (dfc_t*) cal->impl;

    return pinhole_transform(dfc->K, xyz_r);
}

matd_t * dfc_pixels_to_ray(const view_t *view, const matd_t * xy_p)
{
    assert(view->impl_type == VIEW_TYPE);
    const calibration_t *cal = (calibration_t*) view->impl;

    assert(cal->impl_type == CALIBRATION_TYPE);
    const dfc_t *dfc = (dfc_t*) cal->impl;

    matd_t *tmp = pinhole_transform(dfc->Kinv, xy_p);
    matd_t *xyz_r = snap_ray_to_plane(tmp);
    matd_destroy(tmp);

    return xyz_r;
}

void dfc_destroy_view(view_t *view)
{
    assert(view->impl_type == VIEW_TYPE);
    calibration_t *cal = (calibration_t*) view->impl;

    assert(cal->impl_type == CALIBRATION_TYPE);
    dfc_t *dfc = (dfc_t*) cal->impl;

    dfc_destroy(dfc);
}

void dfc_destroy_cal(calibration_t *cal)
{
    assert(cal->impl_type == CALIBRATION_TYPE);
    dfc_t *dfc = (dfc_t*) cal->impl;

    dfc_destroy(dfc);
}

void dfc_destroy(dfc_t *dfc)
{
    free(dfc->view);
    free(dfc->cal);

    matd_destroy(dfc->fc);
    matd_destroy(dfc->cc);
    matd_destroy(dfc->K);
    matd_destroy(dfc->Kinv);

    free(dfc);
}

char* dfc_get_calibration_string(const view_t *view)
{
    assert(view->impl_type == VIEW_TYPE);
    const calibration_t *cal = (calibration_t*) view->impl;

    assert(cal->impl_type == CALIBRATION_TYPE);
    const dfc_t *dfc = (dfc_t*) cal->impl;

    return sprintf_alloc("        class = \"%s\";\n\n"
                         "        width = %d;\n"
                         "        height = %d;\n\n"
                         "        intrinsics {\n"
                         "            fc = [%11.6f,%11.6f ];\n"
                         "            cc = [%11.6f,%11.6f ];\n"
                         "        }\n",
                         "DistortionFreeCalibration",
                         dfc->width,
                         dfc->height,
                         dfc->fc->data[0], dfc->fc->data[1],
                         dfc->cc->data[0], dfc->cc->data[1]);
}

