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

#ifndef _SIM_H
#define _SIM_H

// we support only one collision-type test: ray-cast intersecton.

#include <pthread.h>
#include "common/zarray.h"
#include "common/stype.h"
#include "common/mesh_model.h"
#include "sim_triray.h"

typedef struct sim_world sim_world_t;
struct sim_world
{
    const stype_t *stype;
    zarray_t *objects;
    zarray_t *agents;
    pthread_mutex_t mutex;
    int running;
    int paused;
};

enum SIM_OBJECT_TYPE { SIM_OBJECT_SURFACE = 1, SIM_OBJECT_BOX = 2 };

typedef struct sim_object sim_object_t;
struct sim_object
{
    const stype_t *stype;

    ////////////////////////////////////////////////////////////////////
    // project from local coordinate frame into the global coordinate frame
    double T[16];

    // The point and direction should be transformed by inv(T) by the caller.
    // Should always return [0, max]
    double (*ray_cast)(sim_object_t *obj, const double p[3], const double dir[3], double max);

    ////////////////////////////////////////////////////////////////////
    // This is the model used for appearance purposes. It may have
    // more detailed geometry than that used internally for collision
    // tests. This mesh belongs to the sim_object, who can
    // change/replace it at any call to update.
    mesh_model_t *mesh;

    // For objects that animate their appearance, the renderer may
    // want to cache uploaded resources related to the mesh. For each
    // distinct mesh used, the mesh_version should be unique.
    int64_t mesh_version;

    // If the object cycles between a small set of versions, set the
    // number of versions here. mesh_version should then be less than
    // mesh_nversions. If not used, mesh_nversions can be -1.
    int64_t mesh_nversions;

    ////////////////////////////////////////////////////////////////////
    // t=the time, dt=time since last call to update.
    //
    // The object can modify T, mesh and ray_cast at this time.
    void (*update)(sim_object_t *obj, double t, double dt);

    int type; // SIM_OBJECT_*

    ////////////////////////////////////////////////////////////////////
    // storage private to the sim_object implementation.

    sim_triray_t *triray; // many implementations use this for ray_cast.

    union {
        struct {
            zarray_t *vertices; // float[3]
            zarray_t *normals;   // float[3]
            float     rgba[4];
            float     roughness;

            // rebuilt on deserialization
            zarray_t *tris;       // struct triangle* (TODO, make non-ptr)
        } triangles;

        struct {
            int       width, height;
            double    pitch;
            float     *zs; // width*height

            float     rgba[4];
            float     roughness;
        } surface;

        struct {
            float     M[16];
            float     rgba[4];
            float     roughness;
        } box;
    } u;

    ////////////////////////////////////////////////////////////////////
    // storage private to the sim_world that manages this sim_object.
    struct {
        // XXX replace with hash table
        int     valid;
        int64_t mesh_last_version;
        void    *mesh_last_vxo;
    } world;
};

sim_world_t *sim_world_create();
sim_world_t *sim_world_create_from_file(const char * path);

double sim_world_ray_cast(sim_world_t *sw, const double p[3], const double dir[3], double max,
                          sim_object_t **ignores, int nignores, sim_object_t **out_object);

sim_object_t *sim_object_triangles_create();
void sim_object_triangles_add(sim_object_t *obj, float *a, float *b, float *c);

sim_object_t *sim_object_create_mesh_model(const char *path);

// T: position (always rigid)
// M: model transform (can be non-rigid). identity yields a unit box centered at the origin.
sim_object_t *sim_object_box_create(const double T[16],
                                    const float  M[16],
                                    const float rgba[4], float roughness);

sim_object_t *sim_object_surface_create_dims(const double T[16], double pitch, int width, int height, float *zs,
                                             const float rgba[4], float roughness);

sim_object_t *sim_object_surface_create(const double xy0[2], const double xy1[2], double meters_per_pixel, double z,
                                        const float rgba[4], float roughness);

void sim_object_surface_mound(sim_object_t *so, double x, double y, double dheight, double range);

void sim_stype_init();

void * sim_world_run_thread(void * impl);

#endif
