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

#ifndef RAYTRACER_H
#define RAYTRACER_H

#include "common/image_u8x3.h"

#include "camera.h"
#include "scene.h"


typedef struct {
    camera_t *camera;
    scene_t *scene;
} raytracer_t;


raytracer_t *raytracer_create(camera_t *camera, scene_t *scene);

//Antialias: a value of n means n^2 rays are cast per pixel. must be at least 1
image_u8x3_t *raytracer_render(raytracer_t *r, int width, int height,
                               int antialias, int nthreads);

void raytracer_destroy(raytracer_t *r);

#endif
