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

#ifndef _VIEW_H
#define _VIEW_H

#include "common/matd.h"

#define VIEW_CALIBRATION                         1
#define VIEW_SCALED_VIEW                         2
#define VIEW_MAX_GROWN_INSCRIBED_RECTIFIED_VIEW  3
#define VIEW_MAX_INSCRIBED_RECTIFIED_VIEW        4
#define VIEW_MAX_RECTIFIED_VIEW                  5
#define VIEW_STEREO_RECTIFIED_VIEW               6

typedef struct _view view_t;
struct _view
{
    // type can be a calibration model, or various synthetic views
    int impl_type;
    void* impl;

    // get view dimensions
    int (*get_width)(const view_t *view);
    int (*get_height)(const view_t *view);

    // copy intrinsics matrix
    // note: user frees result
    matd_t * (*copy_intrinsics)(const view_t *view);

    // convert between pixels (distorted if appropriate) and 3D rays.
    // rays are typically on z==1 plane or mag==1 sphere, depending
    // on implementation
    // note: user frees result
    matd_t * (*ray_to_pixels)(const view_t *view, const matd_t * xyz_r);
    matd_t * (*pixels_to_ray)(const view_t *view, const matd_t * xy_p);

    // destroy lowest-level impl and all parents (e.g. view_t, calibration_t, dfc_t)
    void (*destroy)(view_t *view);
};

#endif
