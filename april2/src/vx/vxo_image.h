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

#ifndef _VXO_IMAGE_H
#define _VXO_IMAGE_H

vx_object_t *vxo_image(vx_resource_t *tex);
vx_object_t *vxo_image_saturated(vx_resource_t *tex, float saturation);
vx_object_t *vxo_image_u8x4(image_u8x4_t *im, uint32_t flags);
vx_object_t *vxo_image_u8x3(image_u8x3_t *im, uint32_t flags);
vx_object_t *vxo_image_u8(image_u8_t *im, uint32_t flags);

vx_object_t *vxo_image_tile(vx_resource_t *tex, float rgba0[4], float rgba1[4], float rgba2[4], float rgba3[4]);

// for every non-zero value of the texture, output rgba. Else, discard.
vx_object_t *vxo_image_mask(vx_resource_t *tex, float rgba[4]);

#endif
