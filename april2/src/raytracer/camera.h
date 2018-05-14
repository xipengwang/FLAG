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

#ifndef CAMERA_H
#define CAMERA_H


typedef struct camera {
    double P[16];   // projection matrix
    double M[16];   // model matrix
    double Minv[16];
    double PM[16];  // P * M
    double PMinv[16];
    double pos[3];
} camera_t;


camera_t *camera_create(double fx, double fy, const double eye[3],
                        const double lookat[3], const double up[3]);
void camera_get_ray_direction(camera_t *c, double nx, double ny,
                              double raydir[3]);
void camera_destroy(camera_t *c);

#endif
