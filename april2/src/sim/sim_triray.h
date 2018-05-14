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

#ifndef _SIM_TRI_RAY_H

#include "common/zarray.h"

#include "common/mesh_model.h"

// an object which efficiently supports ray casting queries against a
// set of triangles

typedef struct sim_triray sim_triray_t;
struct sim_triray {

    int    sphere_valid;
    double sphere_xyz[3]; // center of bounding sphere
    double sphere_r2;      // squared radius of sphere

    int type;

    union {
        struct {
            zarray_t *tris; // struct triangle, type 0
        } tris;

        struct {
            zarray_t *strs; // sim_triray_t*, type 1
        } strs;
    } u;
};

sim_triray_t *sim_triray_create();
void sim_triray_destroy(sim_triray_t *str);

void sim_triray_add(sim_triray_t *str, const float p0[3], const float p1[3], const float p2[3]);
void sim_triray_add_mesh(sim_triray_t *str, mesh_model_t *model);

void sim_triray_split(sim_triray_t *str, const double max_sz, const int min_tris);

// dir must be UNIT vector
double sim_triray_cast(sim_triray_t *str, const double p[3], const double dir[3], double max);

#endif
