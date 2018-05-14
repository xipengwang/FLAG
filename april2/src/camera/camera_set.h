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

#ifndef _CAMERA_SET_H
#define _CAMERA_SET_H

#include "common/config.h"
#include "common/matd.h"

#include "calibration.h"

typedef struct _camera_set camera_set_t;

// create a camera set from a config file. the child is the name of
// the calibration block ("<child>.names" should be a valid entry)
camera_set_t * camera_set_create(config_t * config, const char * child);
void camera_set_destroy(camera_set_t *cs);

int camera_set_size(const camera_set_t *cs);

// cs owns results. they should not be modified.
char*          camera_set_get_name(const camera_set_t *cs, int idx);
calibration_t* camera_set_get_calibration(const camera_set_t *cs, int idx);
matd_t*        camera_set_get_extrinsics_B2C(const camera_set_t *cs, int idx); // 4x4 transformation matrix

#endif
