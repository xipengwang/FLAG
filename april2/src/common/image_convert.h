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

#ifndef _IMAGE_CONVERT_H
#define _IMAGE_CONVERT_H

#include "common/image_u8.h"
#include "common/image_u8x3.h"
#include "common/image_u8x4.h"

#include "lcmtypes/image_t.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Convert between image formats. These types are assumed to conform to:
 *   - image_u8:   GRAY
 *   - image_u8x3: RGB
 *   - image_u8x4: RGBA
 * Conversion to grayscale uses the formula: (r+g+g+b)/4
 */

image_u8x3_t *image_u8_to_u8x3(const image_u8_t *in);
image_u8x4_t *image_u8_to_u8x4(const image_u8_t *in);

image_u8_t   *image_u8x3_to_u8(const image_u8x3_t *in);
image_u8x4_t *image_u8x3_to_u8x4(const image_u8x3_t *in);

image_u8_t   *image_u8x4_to_u8(const image_u8x4_t *in);
image_u8x3_t *image_u8x4_to_u8x3(const image_u8x4_t *in);

// Wrappers to convert between image_t and a specified type. Data buffer points
// to the cast input buffer. Use image_t_copy on the wrapper if you want a deep
// copy. Note: An assertion will be thrown if the input image_t does not match
// the types listed above.

const image_t       image_t_wrap_image_u8(const image_u8_t *in, int64_t utime);
const image_u8_t    image_u8_wrap_image_t(const image_t *in);

const image_t       image_t_wrap_image_u8x3(const image_u8x3_t *in, int64_t utime);
const image_u8x3_t  image_u8x3_wrap_image_t(const image_t *in);

const image_t       image_t_wrap_image_u8x4(const image_u8x4_t *in, int64_t utime);
const image_u8x4_t  image_u8x4_wrap_image_t(const image_t *in);

// Conversions that handle raw pixel formats (e.g. RGGB)
image_u8_t   *image_t_to_image_u8(const image_t *in);
image_u8x4_t *image_t_to_image_u8x4(const image_t *in);

// Half-resolution debayering
image_u8x4_t *image_t_to_image_u8x4_half(const image_t *in);

#ifdef __cplusplus
}
#endif

#endif
