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

#include "config_util.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "common/doubles.h"
#include "math_util.h"

char *strcat_safe(const char *s0, const char *s1)
{
    int len0 = strlen(s0);
    int len1 = strlen(s1);
    char *copy = malloc(len0+len1+1);
    strcpy(copy, s0);
    strcat(copy, s1);

    return copy;
}

void config_util_require_pos(const config_t *config, const char *sensor_name, double pos[3])
{
    char *param = strcat_safe(sensor_name, ".position");
    config_require_doubles_len(config, param, pos, 3);
    free(param);
}

void config_util_require_quat(const config_t *config, const char *sensor_name, double q[4])
{
    char *param_quat = strcat_safe(sensor_name, ".quaternion");
    if (config_has_key(config, param_quat)) {
        config_require_doubles_len(config, param_quat, q, 4);
        free(param_quat);
        return;
    }
    free(param_quat);

    char *param_rpyd = strcat_safe(sensor_name, ".rollpitchyaw_degrees");
    if (config_has_key(config, param_rpyd)) {
        double rpy[3];
        config_require_doubles_len(config, param_rpyd, rpy, 3);

        for (int i = 0; i < 3; i++)
            rpy[i] *= M_PI / 180.0;

        doubles_rpy_to_quat(rpy, q);
        free(param_rpyd);
        return;
    }
    free(param_rpyd);

    char *param_rpyr = strcat_safe(sensor_name, ".rollpitchyaw_radians");
    if (config_has_key(config, param_rpyr)) {
        double rpy[3];
        config_require_doubles_len(config, param_rpyr, rpy, 3);

        for (int i = 0; i < 3; i++)
            rpy[i] *= M_PI / 180.0;

        doubles_rpy_to_quat(rpy, q);
        free(param_rpyr);
        return;
    }
    free(param_rpyr);

    printf("No orientation specified for sensor.\n");
    assert(0);
}


void config_util_require_mat(const config_t *config, const char *sensor_name, double M[16])
{
    double q[4], pos[3];

    config_util_require_quat(config, sensor_name, q);
    config_util_require_pos(config, sensor_name, pos);

    doubles_quat_xyz_to_mat44(q, pos, M);
    return;
}
