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

#ifndef _STEREO_RECTIFIED_VIEW_H
#define _STEREO_RECTIFIED_VIEW_H

#include "common/matd.h"

#include "view.h"

typedef struct
{
    view_t *view;

    matd_t * xy01;

    matd_t *K;
    matd_t *Kinv;

    int width;
    int height;
}srv_t;


srv_t *  srv_create(const matd_t *K, const matd_t *XY01);
void     srv_destroy(srv_t *srv);

view_t * srv_get_view(srv_t *srv);

// methods for view interface
int      srv_get_width(const view_t *view);
int      srv_get_height(const view_t *view);
matd_t * srv_copy_intrinsics(const view_t *view);
matd_t * srv_ray_to_pixels(const view_t *view, const matd_t *xyz_r);
matd_t * srv_pixels_to_ray(const view_t *view, const matd_t *xy_rp);
void     srv_destroy_view(view_t *view);

#endif
