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

#ifndef SURFACE_H
#define SURFACE_H

#include <stdbool.h>

#include "common/image_u8.h"


typedef struct surface surface_t;
struct surface {
    double (*get_reflectivity)(surface_t *s, const double pos[3]);
    double (*get_roughness)(surface_t *s, const double pos[3]);

    void (*get_specular_color)(surface_t *s, const double pos[3],
                               float color[3]);
    void (*get_diffuse_color)(surface_t *s, const double pos[3],
                              float color[3]);

    double reflectivity;
    double roughness;
    float specular_color[3];
    float diffuse_color[3];
};

surface_t *surface_constant(const float diffuse_color[3],
                            const float specular_color[3],
                            double reflectivity, double roughness);

surface_t *surface_checkerboard(double reflectivity, double roughness);

surface_t *surface_image_u8(image_u8_t *im, const double xy0[2],
                            const double xy1[2], bool flipy, bool repeat,
                            double reflectivity, double roughness);

surface_t *surface_radial_stripes(const float diffuse_color0[3],
                                  const float diffuse_color1[3],
                                  const float specular_color[3],
                                  double reflectivity, double roughness,
                                  double frequency);

#endif
