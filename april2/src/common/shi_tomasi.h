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

#ifndef SHI_TOMASI
#define SHI_TOMASI

typedef struct {
    image_f32_t * res_img;
    zarray_t * res_points; // 2*int32_t
} shi_tomasi_res_t;

//user owns returned struct and internals.
shi_tomasi_res_t *shi_tomasi_detect_corners(image_f32_t *im, int scale, double sigma, double threshold);
void shi_tomasi_res_t_destroy(shi_tomasi_res_t * res);
#endif
