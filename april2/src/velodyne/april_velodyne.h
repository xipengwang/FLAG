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

#ifndef APRIL_VELODYNE_H
#define APRIL_VELODYNE_H

#include <stdint.h>
#include "common/zarray.h"

#define VELO_FACTORY_HDL_32E            0x21
#define VELO_FACTORY_VLP_16             0x22
#define VELO_FACTORY_STRONGEST_RETURN   0x37
#define VELO_FACTORY_LAST_RETURN        0x38
#define VELO_FACTORY_DUAL_RETURN        0x39

typedef struct april_velodyne april_velodyne_t;

// Create a velodyne object for handling 16 and 32 laser models
april_velodyne_t *april_velodyne_create();
void april_velodyne_destroy(april_velodyne_t *velo);

void april_velodyne_on_packet(april_velodyne_t *velo,
                              const uint8_t *buf,
                              int32_t buflen,
                              float xform[16],
                              void (*on_slice)(const zarray_t *pts, const zarray_t *ranges, const zarray_t *intensities, uint8_t mode, void *user),
                              void (*on_sweep)(void *user),
                              void *user);



#endif
