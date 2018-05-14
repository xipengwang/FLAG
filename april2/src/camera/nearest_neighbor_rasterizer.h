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

#ifndef _NEAREST_NEIGHBOR_RASTERIZER_H
#define _NEAREST_NEIGHBOR_RASTERIZER_H

#include "common/image_u8.h"
#include "common/image_u8x3.h"
#include "common/image_u8x4.h"
#include "common/matd.h"

#include "view.h"

rasterizer_t * nearest_neighbor_rasterizer_create(const view_t *input, const view_t *output);
rasterizer_t * nearest_neighbor_rasterizer_create_rot(const view_t *input,  const matd_t * G2C_input,
                                                      const view_t *output, const matd_t * G2C_output);

void nearest_neighbor_rasterizer_destroy(rasterizer_t *rasterizer);

image_u8_t   *nearest_neighbor_rasterizer_rectify_image_u8(const rasterizer_t   *rasterizer, const image_u8_t   *in);
image_u8x3_t *nearest_neighbor_rasterizer_rectify_image_u8x3(const rasterizer_t *rasterizer, const image_u8x3_t *in);
image_u8x4_t *nearest_neighbor_rasterizer_rectify_image_u8x4(const rasterizer_t *rasterizer, const image_u8x4_t *in);

#endif
