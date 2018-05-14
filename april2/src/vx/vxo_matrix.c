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

#include <math.h>
#include "vx.h"
#include "common/doubles.h"

//////////////////////////////////////////////////////////
static void vxo_matrix_incref(vx_object_t *vxo)
{
    vx_lock();
    vxo->refcnt++;
    vx_unlock();
}

static void vxo_matrix_decref(vx_object_t *vxo)
{
    vx_lock();
    assert(vxo->refcnt > 0);
    int c = --vxo->refcnt;
    vx_unlock();

    if (c == 0) {
        free(vxo->u.matrix.data);
        memset(vxo, 0, sizeof(struct vx_object)); // fail fast
        free(vxo);
    }
}


static void vxo_matrix_serialize(vx_object_t *vxo, vx_serializer_t *outs)
{
    vx_serializer_opcode(outs, VX_SERIALIZER_MODEL_MULTIPLY);

    for (int i = 0; i < 16; i++) {
        vx_serializer_double(outs, vxo->u.matrix.data[i]);
    }
}

vx_object_t *vxo_matrix(const double d[16])
{
    vx_object_t *vxo = calloc(1, sizeof(vx_object_t));
    vxo->u.matrix.data = malloc(16 * sizeof(double));
    memcpy(vxo->u.matrix.data, d, 16 * sizeof(double));
    vxo->serialize = vxo_matrix_serialize;
    vxo->incref = vxo_matrix_incref;
    vxo->decref = vxo_matrix_decref;

    return vxo;
}

vx_object_t *vxo_matrix_scale(double s)
{
    return vxo_matrix( (double[]) { s, 0, 0, 0,
                0, s, 0, 0,
                0, 0, s, 0,
                0, 0, 0, 1 });
}

vx_object_t *vxo_matrix_scale3(double sx, double sy, double sz)
{
    return vxo_matrix( (double[]) { sx, 0, 0, 0,
                0, sy, 0, 0,
                0, 0, sz, 0,
                0, 0, 0, 1 });
}

vx_object_t *vxo_matrix_translate(double tx, double ty, double tz)
{
    return vxo_matrix( (double[]) { 1, 0, 0, tx,
                0, 1, 0, ty,
                0, 0, 1, tz,
                0, 0, 0, 1 });
}

vx_object_t *vxo_matrix_rotatex(double rad)
{
    double s = sin(rad), c = cos(rad);
    return vxo_matrix( (double[]) { 1, 0, 0, 0,
                0, c, -s, 0,
                0, s, c, 0,
                0, 0, 0, 1 });
}

vx_object_t *vxo_matrix_rotatey(double rad)
{
    double s = sin(rad), c = cos(rad);
    return vxo_matrix( (double[]) { c, 0, s, 0,
                0, 1, 0, 0,
                -s, 0, c, 0,
                0, 0, 0, 1 });
}

vx_object_t *vxo_matrix_rotatez(double rad)
{
    double s = sin(rad), c = cos(rad);
    return vxo_matrix( (double[]) { c, -s, 0, 0,
                s, c, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1 });
}

vx_object_t *vxo_matrix_xyt(const double xyt[3])
{
    double s = sin(xyt[2]), c = cos(xyt[2]);
    return vxo_matrix( (double[]) { c, -s, 0, xyt[0],
                s, c, 0, xyt[1],
                0, 0, 1, 0,
                0, 0, 0, 1 });

}

vx_object_t *vxo_matrix_quat(double q[4])
{
    double M[16];
    doubles_quat_to_mat44(q, M);
    return vxo_matrix(M);
}

vx_object_t *vxo_matrix_quat_xyz(double q[4], double xyz[3])
{
    double M[16];
    doubles_quat_xyz_to_mat44(q, xyz, M);
    return vxo_matrix(M);
}
