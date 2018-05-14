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

#ifndef _MAX_RECTIFIED_VIEW_H
#define _MAX_RECTIFIED_VIEW_H

#include "common/config.h"
#include "common/matd.h"

#include "view.h"

typedef struct _mrv mrv_t;

mrv_t *  mrv_create(const view_t * input);
void     mrv_destroy(mrv_t * mrv);

view_t * mrv_get_view(mrv_t * mrv);

matd_t * mrv_compute_xy01(const view_t * input, const matd_t * Krect); // user frees

// methods for view interface
int      mrv_get_width(const view_t *view);
int      mrv_get_height(const view_t *view);
matd_t * mrv_copy_intrinsics(const view_t *view);
matd_t * mrv_ray_to_pixels(const view_t *view, const matd_t * xyz_r);
matd_t * mrv_pixels_to_ray(const view_t *view, const matd_t * xy_p);
void     mrv_destroy_view(view_t *view);

#endif
