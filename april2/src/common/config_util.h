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

#ifndef _CONFIG_UTIL_H
#define _CONFIG_UTIL_H

#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

// expects a subkey ".position"
    void config_util_require_pos(const config_t *config, const char *sensor_name, double pos[3]);

// expects a subkey of EITHER: ".quaternion", ".rollpitchyaw_degrees", ".rollpitchyaw_radians"
    void config_util_require_quat(const config_t *config, const char *sensor_name, double q[4]);

// uses both get_position and get_quaternion to compute a matrix transformation.
    void config_util_require_mat(const config_t *config, const char *sensor_name, double M[16]);

#ifdef __cplusplus
}
#endif

#endif
