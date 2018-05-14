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

#ifndef _CAMERA_H
#define _CAMERA_H

// basic image type
#include "common/image_u8.h"

// basic interfaces
#include "view.h"
#include "calibration.h"
#include "scaled_view.h"

// basic utilities
#include "camera_math.h"
#include "camera_set.h"
#include "distortion_function_verifier.h"
#include "rasterizer.h"

// models
#include "angular_polynomial_calibration.h"
#include "distortion_free_calibration.h"

// tools to compute view dimensions for rectification
#include "max_grown_inscribed_rectified_view.h"
#include "max_inscribed_rectified_view.h"
#include "max_rectified_view.h"
#include "stereo_rectified_view.h"
#include "stereo_rectification.h"

// rasterizers for image resampling
#include "rasterizer.h"
#include "bilinear_rasterizer.h"
#include "nearest_neighbor_rasterizer.h"


#endif
