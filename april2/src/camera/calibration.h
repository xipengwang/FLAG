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

#ifndef _CALIBRATION_H
#define _CALIBRATION_H

#include "common/config.h"

#include "view.h"

#define CALIBRATION_DISTORTION_FREE    1
#define CALIBRATION_ANGULAR_POLYNOMIAL 2

typedef struct _calibration calibration_t;
struct _calibration
{
    // points to super struct. view->impl points to this calibration struct
    view_t *view;

    // types are camera models
    int impl_type;
    void* impl;

    // calibration string
    // note: user frees result
    char* (*get_calibration_string)(const view_t *view);

    // destroy lowest-level impl and all parents (e.g. view_t, calibration_t, dfc_t)
    void (*destroy)(calibration_t *cal);
};

calibration_t * calibration_load_config(config_t *config, const char* child);

#endif
