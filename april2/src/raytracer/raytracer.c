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

#include <float.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <pthread.h>

#include "common/doubles.h"
#include "common/floats.h"
#include "common/image_u8.h"
#include "common/image_u8x3.h"
#include "common/math_util.h"
#include "common/zarray.h"

#include "raytracer.h"


const int MAX_DEPTH = 5;
const double EPS = 0.00001;

typedef struct {
    raytracer_t *r;
    image_u8x3_t *im;
    int y0;    // render lines y0 to (y1-1)
    int y1;
} thread_data_t;


raytracer_t *raytracer_create(camera_t *camera, scene_t *scene)
{
    raytracer_t *r = calloc(1, sizeof(raytracer_t));
    r->camera = camera;
    r->scene = scene;
    return r;
}

void raytracer_destroy(raytracer_t *r)
{
    free(r);
}

static double test_ray(const double ray0[3], const double raydir[3],
                       scene_t *scene)
{
    intersection_t isect = { .dist = DBL_MAX };
    for (int i = 0; i < zarray_size(scene->objects); i += 1) {
        drawable_t *obj;
        zarray_get(scene->objects, i, &obj);

        // Transform rays into object coordinate frame
        double tray0[3];
        double traydir[3];
        doubles_mat44_transform_xyz(obj->Tinv, ray0, tray0);
        doubles_mat44_rotate_vector(obj->Tinv, raydir, traydir);

        // Only updates isect if it's closer than the existing object
        obj->intersect(obj, tray0, traydir, &isect);
    }

    return isect.dist;
}

static void trace_ray(const double ray0[3], const double raydir[3],
                      scene_t *scene, int depth, float color[3])
{
    // Find first object along the ray
    intersection_t isect = { .dist = DBL_MAX };
    for (int i = 0; i < zarray_size(scene->objects); i += 1) {
        drawable_t *obj;
        zarray_get(scene->objects, i, &obj);

        // Transform rays into object coordinate frame
        double tray0[3];
        double traydir[3];
        doubles_mat44_transform_xyz(obj->Tinv, ray0, tray0);
        doubles_mat44_rotate_vector(obj->Tinv, raydir, traydir);

        // Only updates isect if it's closer than the existing object
        obj->intersect(obj, tray0, traydir, &isect);
    }

    // If no intersection, return background color
    if (isect.obj == NULL) {
        floats_copy(scene->bgcolor, color, 3);
    } else {

        // Position of the collision in global coordinates
        double pos[3] = {ray0[0] + raydir[0]*isect.dist,
                         ray0[1] + raydir[1]*isect.dist,
                         ray0[2] + raydir[2]*isect.dist};
        // Project normal into global coordinates
        double normal[3];
        doubles_mat44_rotate_vector(isect.obj->T, isect.normal, normal);
        double dot = doubles_dot(normal, raydir, 3);
        double reflectdir[3] = {raydir[0] - 2*dot*normal[0],
                                raydir[1] - 2*dot*normal[1],
                                raydir[2] - 2*dot*normal[2]};
        doubles_normalize(reflectdir, 3, reflectdir);

        // Compute ambient lighting component
        for (int i = 0; i < 3; i += 1)
            color[i] = isect.diffuse_color[i] * scene->ambient_light[i];

        // Compute direct lighting components (diffuse + specular)
        for (int i = 0; i < zarray_size(scene->lights); i += 1) {
            light_t *light;
            zarray_get(scene->lights, i, &light);

            double light2obj[3];
            doubles_subtract(pos, light->pos, 3, light2obj);
            double light2objdist = doubles_magnitude(light2obj, 3);
            doubles_normalize(light2obj, 3, light2obj);

            double z = test_ray(light->pos, light2obj, scene);
            bool inshadow = (z + EPS) < light2objdist;

            if (!inshadow) {
                double il = max(0, -doubles_dot(light2obj, normal, 3));
                double sp = max(0, -doubles_dot(light2obj, reflectdir, 3));
                sp = pow(sp, isect.roughness);

                for (int j = 0; j < 3; j += 1) {
                    color[j] = (float)(color[j] +
                                   isect.specular_color[j]*light->color[j]*sp +
                                   isect.diffuse_color[j]*light->color[j]*il);
                }
            }
        }

        // Compute reflected component
        double reflectivity = isect.reflectivity;
        if (reflectivity > 0 && depth < MAX_DEPTH) {
            // Jump past the surface we just hit
            for (int i = 0; i < 3; i += 1)
                pos[i] += reflectdir[i] * EPS;
            float reflectcolor[3];
            trace_ray(pos, reflectdir, scene, depth+1, reflectcolor);
            for (int i = 0; i < 3; i += 1)
                color[i] += reflectcolor[i] * reflectivity;
        }
    }

    // Handle fog
    double fogblend = exp(-scene->fog_density * isect.dist);
    for (int i = 0; i < 3; i += 1)
        color[i] = (float)(fogblend * color[i] +
                           (1-fogblend) * scene->fog_color[i]);
}

static void *worker_thread(void *user)
{
    thread_data_t *data = user;
    int y0 = data->y0;
    int y1 = data->y1;
    int width = data->im->width;
    int height = data->im->height;

    for (int y = y0; y < y1; y += 1) {
        double ny = (y - height/2.0) / (height/2.0);
        for (int x = 0; x < width; x += 1) {
            double nx = (x - width/2.0) / (width/2.0);

            double raydir[3];
            camera_get_ray_direction(data->r->camera, nx, ny, raydir);
            float color[3];
            trace_ray(data->r->camera->pos, raydir, data->r->scene, 0, color);
            int offset = (height-1-y)*data->im->stride + 3*x;
            data->im->buf[offset] = imin(255, color[0]*255);
            data->im->buf[offset+1] = imin(255, color[1]*255);
            data->im->buf[offset+2] = imin(255, color[2]*255);
        }
    }

    free(data);
    return NULL;
}

image_u8x3_t *raytracer_render(raytracer_t *r, int width, int height,
                               int antialias, int nthreads)
{
    assert(antialias > 0);

    image_u8x3_t *im = image_u8x3_create(width*antialias, height*antialias);

    // Create render threads
    int y = 0;
    int yincr = im->height / nthreads;
    if (im->height % nthreads) yincr += 1;
    pthread_t threads[nthreads];
    for (int i = 0; i < nthreads; i += 1) {
        thread_data_t *data = calloc(1, sizeof(thread_data_t));
        data->r = r;
        data->im = im;
        data->y0 = y;
        y += yincr;
        data->y1 = imin(y, im->height);
        pthread_create(&threads[i], NULL, worker_thread, data);
    }

    for (int i = 0; i < nthreads; i += 1) {
        pthread_join(threads[i], NULL);
    }

    // Antialias
    if (antialias == 1)
        return im;

    image_u8x3_t *im2 = image_u8x3_create(width, height);
    for (int y = 0; y < height; y += 1) {
        for (int x = 0; x < width; x += 1) {
            double r = 0, g = 0, b = 0;
            for (int iy = y*antialias; iy < (y+1)*antialias; iy += 1) {
                for (int ix = x*antialias; ix < (x+1)*antialias; ix += 1) {
                    int offset = iy*im->stride + 3*ix;
                    r += im->buf[offset];
                    g += im->buf[offset+1];
                    b += im->buf[offset+2];
                }
            }
            int n = antialias*antialias;
            r /= n;
            g /= n;
            b /= n;

            int offset = y*im2->stride + 3*x;
            im2->buf[offset] = r;
            im2->buf[offset+1] = g;
            im2->buf[offset+2] = b;
        }
    }

    image_u8x3_destroy(im);
    return im2;
}
