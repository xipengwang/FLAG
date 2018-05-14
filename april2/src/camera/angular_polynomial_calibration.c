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
#include <math.h>

#include "common/config.h"
#include "common/matd.h"
#include "common/string_util.h"

#include "camera_math.h"
#include "view.h"
#include "calibration.h"
#include "angular_polynomial_calibration.h"

// to make it easier to create new implementations
#define VIEW_TYPE VIEW_CALIBRATION
#define CALIBRATION_TYPE CALIBRATION_ANGULAR_POLYNOMIAL

#define MAX_ITERATIONS 10

struct _apc
{
    view_t *view;
    calibration_t *cal;

    // model parameters
    int width;
    int height;

    matd_t *fc; // always 2x1
    matd_t *cc; // always 2x1
    matd_t *kc; // always Nx1 (variable N)

    matd_t *K;
    matd_t *Kinv;
};

apc_t * apc_create(const matd_t *fc, const matd_t *cc, const matd_t *kc, int width, int height)
{
    assert(fc->nrows*fc->ncols == 2);
    assert(cc->nrows*cc->ncols == 2);
    assert(kc->nrows*kc->ncols >= 0);

    view_t        *view = calloc(1, sizeof(view_t));
    calibration_t *cal  = calloc(1, sizeof(calibration_t));
    apc_t         *apc  = calloc(1, sizeof(apc_t));

    // VIEW
    view->impl_type       = VIEW_TYPE;
    view->impl            = (void*) cal;
    view->get_width       = apc_get_width;
    view->get_height      = apc_get_height;
    view->copy_intrinsics = apc_copy_intrinsics;
    view->ray_to_pixels   = apc_ray_to_pixels;
    view->pixels_to_ray   = apc_pixels_to_ray;
    view->destroy         = apc_destroy_view;

    // CALIBRATION
    cal->view                   = view;
    cal->impl_type              = CALIBRATION_TYPE;
    cal->impl                   = (void*) apc;
    cal->get_calibration_string = apc_get_calibration_string;
    cal->destroy                = apc_destroy_cal;

    // ANGULAR POLYNOMIAL CALIBRATION
    apc->view = view;
    apc->cal  = cal;

    apc->width  = width;
    apc->height = height;

    apc->fc = matd_copy(fc);
    apc->cc = matd_copy(cc);
    apc->kc = matd_copy(kc);

    matd_t *K = matd_create(3, 3);
    K->data[0*3+0] = fc->data[0];
    K->data[1*3+1] = fc->data[1];
    K->data[0*3+2] = cc->data[0];
    K->data[1*3+2] = cc->data[1];
    K->data[2*3+2] = 1;

    apc->K    = K;
    apc->Kinv = matd_inverse(K);

    return apc;
}

apc_t *  apc_config_create(config_t *config, const char* child)
{
    char* key;

    key = sprintf_alloc("%s.intrinsics.fc", child);
    const matd_t * fc = config_require_matd(config, key);
    free(key);

    key = sprintf_alloc("%s.intrinsics.cc", child);
    const matd_t * cc = config_require_matd(config, key);
    free(key);

    key = sprintf_alloc("%s.intrinsics.kc", child);
    const matd_t * kc = config_require_matd(config, key);
    free(key);

    key = sprintf_alloc("%s.width", child);
    int width = config_require_int(config, key);
    free(key);

    key = sprintf_alloc("%s.height", child);
    int height = config_require_int(config, key);
    free(key);

    return apc_create(fc, cc, kc, width, height);
}

view_t * apc_get_view(apc_t *apc)
{
    return apc->view;
}

calibration_t * apc_get_cal(apc_t *apc)
{
    return apc->cal;
}

int apc_get_width(const view_t *view)
{
    assert(view->impl_type == VIEW_TYPE);
    const calibration_t *cal = (calibration_t*) view->impl;

    assert(cal->impl_type == CALIBRATION_TYPE);
    const apc_t *apc = (apc_t*) cal->impl;

    return apc->width;
}

int apc_get_height(const view_t *view)
{
    assert(view->impl_type == VIEW_TYPE);
    const calibration_t *cal = (calibration_t*) view->impl;

    assert(cal->impl_type == CALIBRATION_TYPE);
    const apc_t *apc = (apc_t*) cal->impl;

    return apc->height;
}

matd_t * apc_copy_intrinsics(const view_t *view)
{
    assert(view->impl_type == VIEW_TYPE);
    const calibration_t *cal = (calibration_t*) view->impl;

    assert(cal->impl_type == CALIBRATION_TYPE);
    const apc_t *apc = (apc_t*) cal->impl;

    return matd_copy(apc->K);
}

matd_t * apc_ray_to_pixels(const view_t *view, const matd_t *xyz_r)
{
    assert(view->impl_type == VIEW_TYPE);
    const calibration_t *cal = (calibration_t*) view->impl;

    assert(cal->impl_type == CALIBRATION_TYPE);
    const apc_t *apc = (apc_t*) cal->impl;

    matd_t * xy_dn = apc_distort_ray(apc, xyz_r);
    matd_t * xy_dp = pinhole_transform(apc->K, xy_dn);

    matd_destroy(xy_dn);
    return xy_dp;
}

matd_t * apc_pixels_to_ray(const view_t *view, const matd_t *xy_dp)
{
    assert(view->impl_type == VIEW_TYPE);
    const calibration_t *cal = (calibration_t*) view->impl;

    assert(cal->impl_type == CALIBRATION_TYPE);
    const apc_t *apc = (apc_t*) cal->impl;

    matd_t * xy_dn = pinhole_transform(apc->Kinv, xy_dp);
    matd_t * xyz_r_raw = apc_rectify_to_ray(apc, xy_dn);
    matd_t * xyz_r_sphere = snap_ray_to_sphere(xyz_r_raw);

    matd_destroy(xy_dn);
    matd_destroy(xyz_r_raw);

    return xyz_r_sphere;
}

matd_t * apc_distort_ray(const apc_t * apc, const matd_t * xyz_r)
{
    assert(xyz_r->ncols*xyz_r->nrows == 3);

    double x = xyz_r->data[0];
    double y = xyz_r->data[1];
    double z = xyz_r->data[2];

    double r = sqrt(x*x + y*y);

    double theta  = atan2(r, z);
    double theta2 = theta*theta;

    double psi = atan2(y, x);

    // polynomial mapping function, not to be interpreted as r*theta
    double rtheta   = theta;
    double thetapow = theta;

    int kclen = apc->kc->nrows*apc->kc->ncols;
    for (int i = 0; i < kclen; i++) {
        thetapow *= theta2;
        rtheta   += apc->kc->data[i] * thetapow;
    }

    matd_t * xy_dn = matd_create(2, 1);
    xy_dn->data[0] = rtheta * cos(psi);
    xy_dn->data[1] = rtheta * sin(psi);

    return xy_dn;
}

matd_t * apc_rectify_to_ray(const apc_t * apc, const matd_t * xy_dn)
{
    matd_t * xyz_r = matd_create(3, 1);

    if (matd_vec_mag(xy_dn) < 1e-12) {
        xyz_r->data[0] = 0;
        xyz_r->data[1] = 0;
        xyz_r->data[2] = 1;
        return xyz_r;
    }

    double psi = 0;
    if (fabs(xy_dn->data[1]) > 1e-9)
        psi = atan2(xy_dn->data[1], xy_dn->data[0]);

    double cpsi = cos(psi);
    double spsi = sin(psi);

    double xtheta = xy_dn->data[0] / cpsi;
    double ytheta = xy_dn->data[1] / spsi;

    // avoid division by zero
    double rtheta = 0;
    if (fabs(cpsi) < 1e-9)      rtheta = xtheta;
    else if (fabs(spsi) < 1e-9) rtheta = ytheta;
    else                        rtheta = (xtheta + ytheta) / 2;

    double theta = sqrt(xy_dn->data[0]*xy_dn->data[0] +
                        xy_dn->data[1]*xy_dn->data[1]);

    int kclen = apc->kc->nrows * apc->kc->ncols;
    for (int i = 0; i < MAX_ITERATIONS; i++) {

        double rtheta_higher = 0;
        double thetapow      = theta;
        double theta2        = theta*theta;

        for (int j = 0; j < kclen; j++) {
            thetapow      *= theta2;
            rtheta_higher += apc->kc->data[j]*thetapow;
        }

        theta = rtheta - rtheta_higher;
    }

    // Theta is the angle off of the z axis, which forms a triangle with sides
    // z (adjacent), r (opposite), and mag (hypotenuse). Theta is the hard mapping.
    // Psi, the rotation about the z axis, relates x, y, and r.

    // magnitude == 1

    double z = cos(theta); // cos(theta) = A/H = z/1 = z
    double r = sin(theta); // sin(theta) = O/H = r/1 = r

    double x = r*cos(psi);
    double y = r*sin(psi);

    xyz_r->data[0] = x;
    xyz_r->data[1] = y;
    xyz_r->data[2] = z;
    return xyz_r;
}

void apc_destroy_view(view_t *view)
{
    assert(view->impl_type == VIEW_TYPE);
    calibration_t *cal = (calibration_t*) view->impl;

    assert(cal->impl_type == CALIBRATION_TYPE);
    apc_t *apc = (apc_t*) cal->impl;

    apc_destroy(apc);
}

void apc_destroy_cal(calibration_t *cal)
{
    assert(cal->impl_type == CALIBRATION_TYPE);
    apc_t *apc = (apc_t*) cal->impl;

    apc_destroy(apc);
}

void apc_destroy(apc_t *apc)
{
    free(apc->view);
    free(apc->cal);

    matd_destroy(apc->fc);
    matd_destroy(apc->cc);
    matd_destroy(apc->kc);
    matd_destroy(apc->K);
    matd_destroy(apc->Kinv);

    free(apc);
}

char * apc_get_calibration_string(const view_t *view)
{
    assert(view->impl_type == VIEW_TYPE);
    const calibration_t *cal = (calibration_t*) view->impl;

    assert(cal->impl_type == CALIBRATION_TYPE);
    const apc_t *apc = (apc_t*) cal->impl;

    char* s = sprintf_alloc("        class = \"%s\";\n\n"
                            "        width = %d;\n"
                            "        height = %d;\n\n"
                            "        intrinsics {\n"
                            "            fc = [%11.6f,%11.6f ];\n"
                            "            cc = [%11.6f,%11.6f ];\n"
                            "            kc = [",
                            "AngularPolynomialCalibration",
                            apc->width,
                            apc->height,
                            apc->fc->data[0], apc->fc->data[1],
                            apc->cc->data[0], apc->cc->data[1]);

    int kclen = apc->kc->nrows*apc->kc->ncols;
    for (int i = 0; i < kclen; i++) {
        char* snew = sprintf_alloc("%s%11.6f%s",
                                   s, apc->kc->data[i],
                                   (i+1 < kclen) ? "," : " ];\n");
        free(s);
        s = snew;
    }

    {
        char* snew = sprintf_alloc("%s        }\n", s);
        free(s);
        s = snew;
    }

    return s;
}

