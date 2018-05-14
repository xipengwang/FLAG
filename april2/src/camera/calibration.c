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

#include <stdio.h>
#include <stdlib.h>

#include "common/config.h"
#include "common/string_util.h"

#include "view.h"
#include "calibration.h"
#include "distortion_free_calibration.h"
#include "angular_polynomial_calibration.h"

calibration_t * calibration_load_config(config_t *config, const char* child)
{
    char* key = sprintf_alloc("%s.class", child);
    const char* classname = config_require_string(config, key);
    free(key);

    calibration_t *cal = NULL;

    if (str_ends_with(classname, "DistortionFreeCalibration"))
        cal = dfc_get_cal(dfc_config_create(config, child));
    else if (str_ends_with(classname, "AngularPolynomialCalibration"))
        cal = apc_get_cal(apc_config_create(config, child));

    if (cal == NULL) {
        setlinebuf(stderr);
        fprintf(stderr,
                "Error: calibration_load_config: "
                "calibration '%s' not recognized\n",
                classname);
        exit(1);
    }

    return cal;
}
