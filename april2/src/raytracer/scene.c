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

#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>

#include "common/floats.h"

#include "scene.h"


#define TYPE_SENTINEL     0
#define TYPE_DRAWABLE     1
#define TYPE_LIGHT        2
#define TYPE_MATRIX       3
#define TYPE_CHAIN        4


scene_t *scene_create(const float bgcolor[3], const float ambient_light[3])
{
    scene_t *s = calloc(1, sizeof(scene_t));

    floats_copy(bgcolor, s->bgcolor, 3);
    floats_copy(ambient_light, s->ambient_light, 3);
    s->objects = zarray_create(sizeof(void*));
    s->lights = zarray_create(sizeof(void*));

    return s;
}

static void scene_add_transform(scene_t *scene, scene_object_t so,
                                double T[16])
{
    if (so.type == TYPE_DRAWABLE) {
        drawable_t *d = so.obj;
        doubles_copy(T, d->T, 16);
        doubles_mat44_inv(T, d->Tinv);
        zarray_add(scene->objects, &d);

    } else if (so.type == TYPE_LIGHT) {
        light_t *l = so.obj;
        l->pos[0] = T[3];
        l->pos[1] = T[7];
        l->pos[2] = T[11];
        zarray_add(scene->lights, &l);

    } else if (so.type == TYPE_MATRIX) {
        double Told[16];
        doubles_copy(T, Told, 16);
        doubles_mat_AB(Told, 4, 4, so.obj, 4, 4, T, 4, 4);

        free(so.obj);

    } else if (so.type == TYPE_CHAIN) {
        double T2[16];
        doubles_copy(T, T2, 16);

        zarray_t *objs = so.obj;
        for (int i = 0; i < zarray_size(objs); i += 1) {
            scene_object_t so2;
            zarray_get(objs, i, &so2);
            scene_add_transform(scene, so2, T2);
        }

        zarray_destroy(objs);

    } else {
        printf("Unknown object type: %d\n", so.type);
        assert(0);
    }
}

void scene_add(scene_t *scene, scene_object_t so)
{
    double T[16];
    doubles_mat44_identity(T);
    scene_add_transform(scene, so, T);
}

void scene_destroy(scene_t *s)
{
    for (int i = 0; i < zarray_size(s->objects); i += 1) {
        drawable_t *d;
        zarray_get(s->objects, i, &d);

        d->destroy(d);
    }
    for (int i = 0; i < zarray_size(s->lights); i += 1) {
        light_t *l;
        zarray_get(s->lights, i, &l);

        free(l);
    }
    zarray_destroy(s->objects);
    zarray_destroy(s->lights);

    free(s);
}

scene_object_t so_chain_impl(scene_object_t first, ...)
{
    zarray_t *objs = zarray_create(sizeof(scene_object_t));
    //printf("chain alloc: %p\n", objs);

    va_list ap;
    va_start(ap, first);
    while (first.type != TYPE_SENTINEL) {
        zarray_add(objs, &first);
        first = va_arg(ap, scene_object_t);
    }
    va_end(ap);

    scene_object_t so = {
        .type = TYPE_CHAIN,
        .obj = objs
    };
    return so;
}

scene_object_t so_light(const float color[3])
{
    light_t *l = calloc(1, sizeof(light_t));
    //printf("light alloc: %p\n", l);
    floats_copy(color, l->color, 3);

    scene_object_t so = {
        .type = TYPE_LIGHT,
        .obj = l
    };
    return so;
}

scene_object_t so_matrix(const double T[16])
{
    double *mat = malloc(16*sizeof(double));
    //printf("matrix alloc: %p\n", mat);
    doubles_copy(T, mat, 16);

    scene_object_t so = {
        .type = TYPE_MATRIX,
        .obj = mat
    };
    return so;
}

struct plane {
    double xy0[2];
    double xy1[2];
    surface_t *surface_top;
    surface_t *surface_bottom;
};
static void plane_intersect(drawable_t *obj, const double ray0[3],
                            const double raydir[3], intersection_t *isect)
{
    struct plane *plane = obj->impl;

    // how far away from the plane is ray0?
    double ray0dist = ray0[2];

    // how rapidly are we approaching the plane? (assumes rayDir is unit vector)
    double ddistance = raydir[2];

    // we're on the wrong side of the plane, or we're getting *farther* away from the plane.
    double dist = -ray0dist / ddistance;
    if (fabs(ddistance) < 1e-10 || dist <= 0)
        return;

    double x = ray0[0] + dist*raydir[0];
    double y = ray0[1] + dist*raydir[1];
    if (x < plane->xy0[0] || x > plane->xy1[0] || 
        y < plane->xy0[1] || y > plane->xy1[1])
        return;

    if (dist < isect->dist) {
        // location of collision in object coordinates
        double pos[3] = {x, y, 0};

        if (ray0[2] >= 0) {
            isect->obj = obj;
            isect->dist = dist;

            doubles_copy((double[]){0, 0, 1}, isect->normal, 3);
            surface_t *surface = plane->surface_top;
            isect->roughness = surface->get_roughness(surface, pos);
            isect->reflectivity = surface->get_reflectivity(surface, pos);
            surface->get_specular_color(surface, pos, isect->specular_color);
            surface->get_diffuse_color(surface, pos, isect->diffuse_color);

        } else if (plane->surface_bottom != NULL) {
            isect->obj = obj;
            isect->dist = dist;

            doubles_copy((double[]){0, 0, -1}, isect->normal, 3);
            surface_t *surface = plane->surface_bottom;
            isect->roughness = surface->get_roughness(surface, pos);
            isect->reflectivity = surface->get_reflectivity(surface, pos);
            surface->get_specular_color(surface, pos, isect->specular_color);
            surface->get_diffuse_color(surface, pos, isect->diffuse_color);
        }
    }
}

static void plane_destroy(drawable_t *obj)
{
    //printf("plane free: %p\n", obj);
    struct plane *plane = obj->impl;
    free(plane->surface_top);
    free(plane->surface_bottom);
    free(plane);
    free(obj);
}

scene_object_t so_plane(surface_t *surface_top, surface_t *surface_bottom,
                        const double xy0[2], const double xy1[2])
{
    drawable_t *d = calloc(1, sizeof(drawable_t));
    struct plane *plane = calloc(1, sizeof(struct plane));
    //printf("plane alloc: %p\n", d);
    doubles_copy(xy0, plane->xy0, 2);
    doubles_copy(xy1, plane->xy1, 2);
    plane->surface_top = surface_top;
    plane->surface_bottom = surface_bottom;
    d->impl = plane;
    d->intersect = plane_intersect;
    d->destroy = plane_destroy;

    scene_object_t so = {
        .type = TYPE_DRAWABLE,
        .obj = d
    };
    return so;
}


struct sphere {
    surface_t *surface;
    double radius;
};

static void sphere_intersect(drawable_t *obj, const double ray0[3],
                            const double raydir[3], intersection_t *isect)
{
    struct sphere *sphere = obj->impl;

    double dx = -ray0[0];
    double dy = -ray0[1];
    double dz = -ray0[2];

    double dot = -doubles_dot(ray0, raydir, 3);
    double dist = 0;

    if (dot >= 0) {
        double disc = sphere->radius*sphere->radius -
            (dx*dx + dy*dy + dz*dz - dot*dot);
        if (disc >= 0)
            dist = dot - sqrt(disc);
    }
    if (dist == 0)
        return;

    if (dist < isect->dist) {
        double pos[3] = {ray0[0] + dist*raydir[0],
                         ray0[1] + dist*raydir[1],
                         ray0[2] + dist*raydir[2]};
        isect->obj = obj;
        isect->dist = dist;
        surface_t *surface = sphere->surface;
        isect->roughness = surface->get_roughness(surface, pos);
        isect->reflectivity = surface->get_reflectivity(surface, pos);
        surface->get_specular_color(surface, pos, isect->specular_color);
        surface->get_diffuse_color(surface, pos, isect->diffuse_color);

        doubles_normalize(pos, 3, pos);
        doubles_copy(pos, isect->normal, 3);
    }
}

static void sphere_destroy(drawable_t *obj)
{
    //printf("sphere free: %p\n", obj);
    struct sphere *sphere = obj->impl;
    free(sphere->surface);
    free(sphere);
    free(obj);
}

scene_object_t so_sphere(surface_t *surface, double radius)
{
    drawable_t *d = calloc(1, sizeof(drawable_t));
    struct sphere *sphere = calloc(1, sizeof(struct sphere));
    //printf("sphere alloc: %p\n", d);
    sphere->surface = surface;
    sphere->radius = radius;
    d->impl = sphere;
    d->intersect = sphere_intersect;
    d->destroy = sphere_destroy;

    scene_object_t so = {
        .type = TYPE_DRAWABLE,
        .obj = d
    };
    return so;
}
