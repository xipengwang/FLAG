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

#ifndef SCENE_H
#define SCENE_H

#include "common/doubles.h"
#include "common/zarray.h"

#include "surface.h"

typedef struct scene scene_t;
typedef struct scene_object scene_object_t;
typedef struct intersection intersection_t;
typedef struct drawable drawable_t;
typedef struct light light_t;

struct scene {
    float bgcolor[3];
    float ambient_light[3];

    float fog_color[3];
    double fog_density;

    zarray_t *objects;   // array of drawable_t*
    zarray_t *lights;    // array of light_t*
};

// Essentially a void pointer tagged with a type. small enough to pass by value
struct scene_object {
    void *obj;
    int type;
};

struct intersection {
    drawable_t *obj;

    double dist;
    double normal[3];  // in object coordinate frame
    double roughness;
    double reflectivity;
    float specular_color[3];
    float diffuse_color[3];
};

// NB: the class SceneObject has been renamed drawable_t
struct drawable {
    void (*intersect)(drawable_t *obj, const double ray0[3],
                      const double raydir[3], intersection_t *isect);
    void (*destroy)(drawable_t *obj);

    double T[16];     // object to world
    double Tinv[16];  // world to object
    void *impl;
};

struct light {
    double pos[3];
    float color[3];
};


scene_t *scene_create(const float bgcolor[3], const float ambient_light[3]);

void scene_add(scene_t *scene, scene_object_t so);

void scene_destroy(scene_t *s);

// Below, different types of scene objects are defined

// NOTE: scene_object_t's should not be reused
// They are either freed by scene_destroy (for objects and lights)
// or upon the return of scene_add (for chains and matrices).

#define SO_SENTINEL ((scene_object_t){.type = 0})
#define so_chain(...) so_chain_impl(__VA_ARGS__, SO_SENTINEL)
scene_object_t so_chain_impl(scene_object_t first, ...);

scene_object_t so_light(const float color[3]);

scene_object_t so_matrix(const double T[16]);

static inline
scene_object_t so_matrix_translate(double tx, double ty, double tz)
{
    double T[16];
    doubles_mat44_translate((double[]){tx, ty, tz}, T);
    return so_matrix(T);
}

static inline
scene_object_t so_matrix_rotatex(double rad)
{
    double T[16];
    doubles_mat44_rotate_x(rad, T);
    return so_matrix(T);
}

static inline
scene_object_t so_matrix_rotatey(double rad)
{
    double T[16];
    doubles_mat44_rotate_y(rad, T);
    return so_matrix(T);
}

static inline
scene_object_t so_matrix_rotatez(double rad)
{
    double T[16];
    doubles_mat44_rotate_z(rad, T);
    return so_matrix(T);
}

scene_object_t so_plane(surface_t *surface_top, surface_t *surface_bottom,
                        const double xy0[2], const double xy1[2]);

scene_object_t so_sphere(surface_t *surface, double radius);

#endif
