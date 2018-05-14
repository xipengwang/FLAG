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

#include <stdio.h>

#include "common/doubles.h"

#include "camera.h"

void mat44_inv_general(const double A[16], double X[16])
{
    double m00 = A[0], m01 = A[1], m02 = A[2], m03 = A[3];
    double m10 = A[4], m11 = A[5], m12 = A[6], m13 = A[7];
    double m20 = A[8], m21 = A[9], m22 = A[10], m23 = A[11];
    double m30 = A[12], m31 = A[13], m32 = A[14], m33 = A[15];

    double det = m00 * m11 * m22 * m33 - m00 * m11 * m23 * m32 -
        m00 * m21 * m12 * m33 + m00 * m21 * m13 * m32 +
        m00 * m31 * m12 * m23 -
        m00 * m31 * m13 * m22 - m10 * m01 * m22 * m33 +
        m10 * m01 * m23 * m32 + m10 * m21 * m02 * m33 -
        m10 * m21 * m03 * m32 - m10 * m31 * m02 * m23 +
        m10 * m31 * m03 * m22 + m20 * m01 * m12 * m33 -
        m20 * m01 * m13 * m32 - m20 * m11 * m02 * m33 +
        m20 * m11 * m03 * m32 + m20 * m31 * m02 * m13 -
        m20 * m31 * m03 * m12 - m30 * m01 * m12 * m23 +
        m30 * m01 * m13 * m22 + m30 * m11 * m02 * m23 -
        m30 * m11 * m03 * m22 - m30 * m21 * m02 * m13 +
        m30 * m21 * m03 * m12;

    if (fabs(det) < 1e-10) {
        printf("Error: matrix not invertible\n");
        return;
    }

    X[0] =   m11 * m22 * m33 - m11 * m23 * m32 - m21 * m12 * m33 + m21 * m13 * m32 + m31 * m12 * m23 - m31 * m13 * m22;
    X[4] = - m10 * m22 * m33 + m10 * m23 * m32 + m20 * m12 * m33 - m20 * m13 * m32 - m30 * m12 * m23 + m30 * m13 * m22;
    X[8] =   m10 * m21 * m33 - m10 * m23 * m31 - m20 * m11 * m33 + m20 * m13 * m31 + m30 * m11 * m23 - m30 * m13 * m21;
    X[12] = - m10 * m21 * m32 + m10 * m22 * m31 + m20 * m11 * m32 - m20 * m12 * m31 - m30 * m11 * m22 + m30 * m12 * m21;
    X[1] = - m01 * m22 * m33 + m01 * m23 * m32 + m21 * m02 * m33 - m21 * m03 * m32 - m31 * m02 * m23 + m31 * m03 * m22;
    X[5] =   m00 * m22 * m33 - m00 * m23 * m32 - m20 * m02 * m33 + m20 * m03 * m32 + m30 * m02 * m23 - m30 * m03 * m22;
    X[9] = - m00 * m21 * m33 + m00 * m23 * m31 + m20 * m01 * m33 - m20 * m03 * m31 - m30 * m01 * m23 + m30 * m03 * m21;
    X[13] =   m00 * m21 * m32 - m00 * m22 * m31 - m20 * m01 * m32 + m20 * m02 * m31 + m30 * m01 * m22 - m30 * m02 * m21;
    X[2] =   m01 * m12 * m33 - m01 * m13 * m32 - m11 * m02 * m33 + m11 * m03 * m32 + m31 * m02 * m13 - m31 * m03 * m12;
    X[6] = - m00 * m12 * m33 + m00 * m13 * m32 + m10 * m02 * m33 - m10 * m03 * m32 - m30 * m02 * m13 + m30 * m03 * m12;
    X[10] =   m00 * m11 * m33 - m00 * m13 * m31 - m10 * m01 * m33 + m10 * m03 * m31 + m30 * m01 * m13 - m30 * m03 * m11;
    X[14] = - m00 * m11 * m32 + m00 * m12 * m31 + m10 * m01 * m32 - m10 * m02 * m31 - m30 * m01 * m12 + m30 * m02 * m11;
    X[3] = - m01 * m12 * m23 + m01 * m13 * m22 + m11 * m02 * m23 - m11 * m03 * m22 - m21 * m02 * m13 + m21 * m03 * m12;
    X[7] =   m00 * m12 * m23 - m00 * m13 * m22 - m10 * m02 * m23 + m10 * m03 * m22 + m20 * m02 * m13 - m20 * m03 * m12;
    X[11] = - m00 * m11 * m23 + m00 * m13 * m21 + m10 * m01 * m23 - m10 * m03 * m21 - m20 * m01 * m13 + m20 * m03 * m11;
    X[15] =   m00 * m11 * m22 - m00 * m12 * m21 - m10 * m01 * m22 + m10 * m02 * m21 + m20 * m01 * m12 - m20 * m02 * m11;

    for (int i = 0; i < 16; i++)
        X[i] /= det;
}

camera_t *camera_create(double fx, double fy, const double eye[3],
                        const double lookat[3], const double _up[3])
{
    camera_t *c = calloc(1, sizeof(camera_t));

    doubles_copy(eye, c->pos, 3);

    double zf = 100000, zn = 10;
    // P is the intrinsics matrix
    double P[16] = {fx, 0, 0, 0,
                    0, fy, 0, 0,
                    0, 0, (zf+zn)/(zn-zf), 2*zf*zn/(zn-zf),
                    0, 0, -1, 0};
    doubles_copy(P, c->P, 16);

    double up[3];
    doubles_normalize(_up, 3, up);
    double f[3];
    doubles_subtract(lookat, eye, 3, f);
    doubles_normalize(f, 3, f);

    double s[3];
    double u[3];
    doubles_cross_product(f, up, s);
    doubles_cross_product(s, f, u);

    double R[16] = {s[0], s[1], s[2], 0,
                    u[0], u[1], u[2], 0,
                    -f[0], -f[1], -f[2], 0,
                    0, 0, 0, 1};

    double T[16] = {1, 0, 0, -eye[0],
                    0, 1, 0, -eye[1],
                    0, 0, 1, -eye[2],
                    0, 0, 0, 1};

    // M is the extrinsics matrix [R | t] where t = -R*c
    doubles_mat_AB(R, 4, 4, T, 4, 4, c->M, 4, 4);
    mat44_inv_general(c->M, c->Minv);
    // PM is the projection matrix = intrinsics * extrinsics
    doubles_mat_AB(c->P, 4, 4, c->M, 4, 4, c->PM, 4, 4);
    mat44_inv_general(c->PM, c->PMinv);

    //printf("(PM)^-1:\n");
    //doubles_print_mat(c->PMinv, 4, 4, "%10f");

    return c;
}

void camera_get_ray_direction(camera_t *c, double nx, double ny,
                              double raydir[3])
{
    // Unproject [nx ny 1 1]'
    double x[4] = {nx, ny, 1, 1};
    double tmp[4];
    doubles_mat_Ab(c->PMinv, 4, 4, x, 4, tmp, 4);
    for (int i = 0; i < 4; i += 1) {
        tmp[i] /= tmp[3];
    }

    doubles_subtract(tmp, c->pos, 3, raydir);
    doubles_normalize(raydir, 3, raydir);
}

void camera_destroy(camera_t *c)
{
    free(c);
}
