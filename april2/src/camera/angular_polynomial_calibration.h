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

#ifndef _ANGULAR_POLYNOMIAL_CALIBRATION_H
#define _ANGULAR_POLYNOMIAL_CALIBRATION_H

#include "common/config.h"
#include "common/matd.h"

#include "view.h"
#include "calibration.h"

typedef struct _apc apc_t;

// apc methods. matd_t objects are copied internally
apc_t *  apc_create(const matd_t *fc, const matd_t *cc, const matd_t *kc, int width, int height);
apc_t *  apc_config_create(config_t *config, const char* child);
void     apc_destroy(apc_t *apc);

view_t        * apc_get_view(apc_t *apc);
calibration_t * apc_get_cal(apc_t *apc);

matd_t * apc_distort_ray(const apc_t * apc, const matd_t * xyz_r);
matd_t * apc_rectify_to_ray(const apc_t * apc, const matd_t * xy_dn);

// methods for view.h. user frees matd_t objects
int      apc_get_width(const view_t *view);
int      apc_get_height(const view_t *view);
matd_t * apc_copy_intrinsics(const view_t *view);
matd_t * apc_ray_to_pixels(const view_t *view, const matd_t *xyz_r);
matd_t * apc_pixels_to_ray(const view_t *view, const matd_t *xy_p);
void     apc_destroy_view(view_t *view);

// methods for calibration.h. user frees char*
char*    apc_get_calibration_string(const view_t *view);
void     apc_destroy_cal(calibration_t *cal);

#endif
