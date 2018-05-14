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

#ifndef _DISTORTION_FREE_CALIBRATION_H
#define _DISTORTION_FREE_CALIBRATION_H

#include "common/config.h"
#include "common/matd.h"

#include "view.h"
#include "calibration.h"

typedef struct _dfc dfc_t;

// dfc methods. matd_t objects are copied
dfc_t *  dfc_create(const matd_t *fc, const matd_t * cc, int width, int height);
dfc_t *  dfc_config_create(config_t *config, const char* child);
void     dfc_destroy(dfc_t *dfc);

view_t        * dfc_get_view(dfc_t *dfc);
calibration_t * dfc_get_cal(dfc_t *dfc);

// methods for view.h. user frees matd_t objects
int      dfc_get_width(const view_t *view);
int      dfc_get_height(const view_t *view);
matd_t * dfc_copy_intrinsics(const view_t *view);
matd_t * dfc_ray_to_pixels(const view_t *view, const matd_t * xyz_r);
matd_t * dfc_pixels_to_ray(const view_t *view, const matd_t * xy_p);
void     dfc_destroy_view(view_t *view);

// Methods for calibration.h. user frees char*
char*    dfc_get_calibration_string(const view_t *view);
void     dfc_destroy_cal(calibration_t *cal);

#endif
