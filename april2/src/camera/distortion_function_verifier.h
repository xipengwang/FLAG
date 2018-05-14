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

#ifndef _DISTORTION_FUNCTION_VERIFIER_H
#define _DISTORTION_FUNCTION_VERIFIER_H

#include "common/matd.h"
#include "common/zarray.h"

#include "view.h"

typedef struct _dfv dfv_t;

// create a DFV. recommended params: radius_buffer: 0.10, dtheta: pi/1000
dfv_t *  dfv_create(const view_t *view, double radius_buffer, double dtheta);
void     dfv_destroy(dfv_t *dfv);

// return 1 if valid, 0 if invalid
int      dfv_ray_valid(const dfv_t *dfv, const matd_t *xyz_r);
int      dfv_pixel_valid(const dfv_t *dfv, const matd_t *xy_dp);

// user frees result
matd_t * dfv_clamp_ray(const dfv_t *dfv, const matd_t *xyz_r);
matd_t * dfv_clamp_pixels(const dfv_t *dfv, const matd_t *xy_dp);

// undistort the image border, taking steps around the border of the specified size
// (recommended stepsize: 1). the view is used to undistort and K is used to reproject.
// K can be a 3x3 intrinsics-rotation product if desired (K*R)
zarray_t * dfv_compute_rectified_border(const dfv_t *dfv, const view_t * view, const matd_t * K, double stepsize);

#endif
