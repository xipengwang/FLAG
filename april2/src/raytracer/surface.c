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

#include <limits.h>

#include "common/floats.h"
#include "common/doubles.h"

#include "surface.h"


static double default_get_reflectivity(surface_t *s, const double pos[3])
{
    return s->reflectivity;
}

static double default_get_roughness(surface_t *s, const double pos[3])
{
    return s->roughness;
}

static void default_get_specular_color(surface_t *s, const double pos[3],
                                       float color[3])
{
    floats_copy(s->specular_color, color, 3);
}

static void default_get_diffuse_color(surface_t *s, const double pos[3],
                                      float color[3])
{
    floats_copy(s->diffuse_color, color, 3);
}

static inline void surface_default_init(surface_t *s)
{
    s->diffuse_color[0] = 1.0f;
    s->diffuse_color[1] = 1.0f;
    s->diffuse_color[2] = 1.0f;
    s->specular_color[0] = 1.0f;
    s->specular_color[1] = 1.0f;
    s->specular_color[2] = 1.0f;
    s->get_reflectivity = default_get_reflectivity;
    s->get_roughness = default_get_roughness;
    s->get_specular_color = default_get_specular_color;
    s->get_diffuse_color = default_get_diffuse_color;
}

surface_t *surface_constant(const float diffuse_color[3],
                            const float specular_color[3],
                            double reflectivity, double roughness)
{
    surface_t *surf = calloc(1, sizeof(surface_t));
    surface_default_init(surf);

    surf->reflectivity = reflectivity;
    surf->roughness = roughness;
    floats_copy(diffuse_color, surf->diffuse_color, 3);
    floats_copy(specular_color, surf->specular_color, 3);

    return surf;
}


typedef struct {
    surface_t s;  // KEEP ME FIRST

    float alt_color[3];
    double scale;
    int axis0;
    int axis1;
} surface_checkerboard_t;

static void checkerboard_get_diffuse_color(surface_t *s, const double pos[3],
                                           float color[3])
{
    surface_checkerboard_t *c = (surface_checkerboard_t *)s;
    int v = (int)(c->scale * pos[c->axis0] + INT_MAX/2) +
        (int)(c->scale * pos[c->axis1] + INT_MAX/2);
    if (v & 1)
        floats_copy(c->s.diffuse_color, color, 3);
    else
        floats_copy(c->alt_color, color, 3);
}

surface_t *surface_checkerboard(double reflectivity, double roughness)
{
    surface_checkerboard_t *surf = calloc(1, sizeof(surface_checkerboard_t));
    surface_default_init(&surf->s);
    surf->s.get_diffuse_color = checkerboard_get_diffuse_color;

    surf->s.reflectivity = reflectivity;
    surf->s.roughness = roughness;

    surf->scale = 1;
    surf->axis0 = 0;
    surf->axis1 = 1;

    // diffuse_color is odd
    // alt_color is even
    surf->alt_color[0] = 0.1f;
    surf->alt_color[1] = 0.1f;
    surf->alt_color[2] = 0.1f;

    return (surface_t *)surf;
}


typedef struct {
    surface_t s;  // KEEP ME FIRST

    image_u8_t *im;
    double xy0[2];
    double xy1[2];
    bool repeat;
    bool flipy;
    double scale;
} surface_image_t;

static void image_get_diffuse_color(surface_t *s, const double pos[3],
                                    float color[3])
{
    surface_image_t *si = (surface_image_t *)s;

    double x = (pos[0] - si->xy0[0]) / (si->xy1[0] - si->xy0[0]);
    double y = (pos[1] - si->xy0[1]) / (si->xy1[1] - si->xy0[1]);

    int ix = (int)(x * si->im->width);
    int iy = (int)(y * si->im->height);

    if (ix >= 0 && ix < si->im->width &&
        iy >= 0 && iy < si->im->height) {
        if (si->flipy)
            iy = si->im->height - 1 - iy;

        int offset = iy*si->im->stride + ix;
        color[0] = color[1] = color[2] = si->im->buf[offset] / 255.0f;
    } else {
        floats_copy(s->diffuse_color, color, 3);
    }
}

surface_t *surface_image_u8(image_u8_t *im, const double xy0[2],
                            const double xy1[2], bool flipy, bool repeat,
                            double reflectivity, double roughness)
{
    surface_image_t *surf = calloc(1, sizeof(surface_image_t));
    surface_default_init(&surf->s);
    surf->s.get_diffuse_color = image_get_diffuse_color;

    surf->s.reflectivity = reflectivity;
    surf->s.roughness = roughness;

    surf->im = im;
    surf->flipy = flipy;
    surf->repeat = repeat;
    doubles_copy(xy0, surf->xy0, 2);
    doubles_copy(xy1, surf->xy1, 2);

    return (surface_t *)surf;
}


typedef struct {
    surface_t s;  // KEEP ME FIRST

    float diffuse_color1[3];
    double frequency;
} surface_radial_stripes_t;

static void radial_stripes_get_diffuse_color(surface_t *s, const double pos[3],
                                             float color[3])
{
    surface_radial_stripes_t *sr = (surface_radial_stripes_t *)s;

    double theta = atan2(pos[1], pos[0]);
    double a = sin(sr->frequency*theta)/2 + 0.5;
    double b = 1-a;
    for (int i = 0; i < 3; i += 1)
        color[i] = (float)(a*s->diffuse_color[i] + b*sr->diffuse_color1[i]);
}

surface_t *surface_radial_stripes(const float diffuse_color0[3],
                                  const float diffuse_color1[3],
                                  const float specular_color[3],
                                  double reflectivity, double roughness,
                                  double frequency)
{
    surface_radial_stripes_t *surf =
        calloc(1, sizeof(surface_radial_stripes_t));
    surface_default_init(&surf->s);
    surf->s.get_diffuse_color = radial_stripes_get_diffuse_color;

    floats_copy(diffuse_color0, surf->s.diffuse_color, 3);
    floats_copy(diffuse_color1, surf->diffuse_color1, 3);
    floats_copy(specular_color, surf->s.specular_color, 3);
    surf->s.reflectivity = reflectivity;
    surf->s.roughness = roughness;
    surf->frequency = frequency;

    return (surface_t *)surf;
}
