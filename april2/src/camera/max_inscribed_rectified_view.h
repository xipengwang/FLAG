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

#ifndef _MAX_INSCRIBED_RECTIFIED_VIEW_H
#define _MAX_INSCRIBED_RECTIFIED_VIEW_H

#include "common/config.h"
#include "common/matd.h"

#include "view.h"

typedef struct _mirv mirv_t;

mirv_t *  mirv_create(const view_t * input);
void      mirv_destroy(mirv_t * mirv);

view_t *  mirv_get_view(mirv_t * mirv);

matd_t *  mirv_compute_xy01(const view_t *input, const matd_t * Krect); // user frees
matd_t *  mirv_get_bounds_xy01(const mirv_t * mirv); // user frees

// methods for view interface
int       mirv_get_width(const view_t *view);
int       mirv_get_height(const view_t *view);
matd_t *  mirv_copy_intrinsics(const view_t *view);
matd_t *  mirv_ray_to_pixels(const view_t *view, const matd_t * xyz_r);
matd_t *  mirv_pixels_to_ray(const view_t *view, const matd_t * xy_p);
void      mirv_destroy_view(view_t *view);

#endif

