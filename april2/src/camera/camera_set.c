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

#include <stdlib.h>
#include <math.h>
#include <assert.h>

#include "common/config.h"
#include "common/matd.h"
#include "common/doubles.h"
#include "common/string_util.h"
#include "common/zarray.h"

#include "calibration.h"
#include "camera_set.h"

struct _camera_set
{
    int num_cameras;
    zarray_t *names;          // type: char*
    zarray_t *calibrations;   // type: calibration_t*
    zarray_t *B2C_extrinsics; // type: matd_t* (4x4)
};

camera_set_t * camera_set_create(config_t * config, const char * child)
{
    char* key = NULL;

    camera_set_t *cs = calloc(1, sizeof(camera_set_t));

    key = sprintf_alloc("%s.names", child);
    const zarray_t *names = config_require_strings(config, key);
    free(key);

    cs->num_cameras    = zarray_size(names);
    cs->names          = zarray_create(sizeof(char*));
    cs->calibrations   = zarray_create(sizeof(calibration_t*));
    cs->B2C_extrinsics = zarray_create(sizeof(matd_t*));

    for (int i = 0; i < cs->num_cameras; i++) {

        char* name = NULL;
        zarray_get(names, i, &name);
        char* camchild = sprintf_alloc("%s.%s", child, name);

        // copy name (config owns original)
        char *dup = strdup(name);
        zarray_add(cs->names, &dup);

        // load camera model
        calibration_t *cal = calibration_load_config(config, camchild);
        zarray_add(cs->calibrations, &cal);

        // load extrinsics
        key = sprintf_alloc("%s.extrinsics.position", camchild);
        const matd_t *xyz = config_require_matd(config, key);
        free(key);

        key = sprintf_alloc("%s.extrinsics.rollpitchyaw_degrees", camchild);
        const matd_t *rpy_deg = config_require_matd(config, key);
        free(key);

        // convert extrinsics to xyz (m) rpy (radians)
        matd_t *B2C_xyzrpy = matd_create(6, 1);

        B2C_xyzrpy->data[0] = xyz->data[0];
        B2C_xyzrpy->data[1] = xyz->data[1];
        B2C_xyzrpy->data[2] = xyz->data[2];
        B2C_xyzrpy->data[3] = rpy_deg->data[0]*M_PI/180.0;
        B2C_xyzrpy->data[4] = rpy_deg->data[1]*M_PI/180.0;
        B2C_xyzrpy->data[5] = rpy_deg->data[2]*M_PI/180.0;

        // convert to matrix
        matd_t *B2C = matd_create(4, 4);
        doubles_xyzrpy_to_mat44(B2C_xyzrpy->data, B2C->data);
//        ssc_homo4x4(B2C->data, B2C_xyzrpy->data);
        matd_destroy(B2C_xyzrpy);

        zarray_add(cs->B2C_extrinsics, &B2C);

        free(camchild);
    }

    assert(cs->num_cameras == zarray_size(cs->names));
    assert(cs->num_cameras == zarray_size(cs->calibrations));
    assert(cs->num_cameras == zarray_size(cs->B2C_extrinsics));

    return cs;
}

void camera_set_destroy(camera_set_t *cs)
{
    // names
    zarray_vmap(cs->names, free);
    zarray_destroy(cs->names);

    // calibrations
    for (int i = 0; i < cs->num_cameras; i++) {
        calibration_t *cal = NULL;
        zarray_get(cs->calibrations, i, &cal);
        cal->destroy(cal);
    }
    zarray_destroy(cs->calibrations);

    // extrinsics
    //zarray_vmap(cs->B2C_extrinsics, matd_destroy);
    zarray_destroy(cs->B2C_extrinsics);

    // camera set
    free(cs);
}

int camera_set_size(const camera_set_t *cs)
{
    return cs->num_cameras;
}

char* camera_set_get_name(const camera_set_t *cs, int idx)
{
    assert(idx >= 0 && idx < cs->num_cameras);
    char *name = NULL;
    zarray_get(cs->names, idx, &name);
    return name;
}

calibration_t* camera_set_get_calibration(const camera_set_t *cs, int idx)
{
    assert(idx >= 0 && idx < cs->num_cameras);
    calibration_t *cal = NULL;
    zarray_get(cs->calibrations, idx, &cal);
    return cal;
}

matd_t* camera_set_get_extrinsics_B2C(const camera_set_t *cs, int idx)
{
    assert(idx >= 0 && idx < cs->num_cameras);
    matd_t *m = NULL;
    zarray_get(cs->B2C_extrinsics, idx, &m);
    return m;
}
