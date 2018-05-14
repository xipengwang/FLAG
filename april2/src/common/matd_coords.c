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

#include "matd_coords.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdlib.h>

// multi-dimension element access macro
#define EL(m, row,col) (m)->data[((row)*(m)->ncols + (col))]

// vector (single-dimension) element access macro
// Note: this macro supports both column-based, row-based vectors
#define ELV(m, i) (m)->data[(i)]

#ifndef M_PI
# define M_PI 3.141592653589793238462643383279502884197169399
#endif

static inline int is_vector_len(const matd_t * a, int len)
{
    return (a->ncols == 1 && a->nrows == len) || (a->ncols == len && a->nrows == 1);
}

matd_t * matd_quat_rotate(const matd_t * qm, const matd_t * v3m)
{
    assert(is_vector_len(qm, 4));
    assert(is_vector_len(v3m, 3));

    // Ported from april.jmat.LinAlg
    double t2, t3, t4, t5, t6, t7, t8, t9, t10;

    t2 = EL(qm,0,0)*EL(qm,1,0);
    t3 = EL(qm,0,0)*EL(qm,2,0);
    t4 = EL(qm,0,0)*EL(qm,3,0);
    t5 = -EL(qm,1,0)*EL(qm,1,0);
    t6 = EL(qm,1,0)*EL(qm,2,0);
    t7 = EL(qm,1,0)*EL(qm,3,0);
    t8 = -EL(qm,2,0)*EL(qm,2,0);
    t9 = EL(qm,2,0)*EL(qm,3,0);
    t10 = -EL(qm,3,0)*EL(qm,3,0);

    matd_t * out = matd_create(3,1);
    EL(out, 0,0) = 2*((t8+t10)* EL(v3m,0,0) + (t6-t4)*EL(v3m,1,0)  + (t3+t7)*EL(v3m,2,0)) + EL(v3m,0,0);
    EL(out, 1,0) = 2*((t4+t6)*EL(v3m,0,0)  + (t5+t10)*EL(v3m,1,0) + (t9-t2)*EL(v3m,2,0)) + EL(v3m,1,0);
    EL(out, 2,0) = 2*((t7-t3)*EL(v3m,0,0)  + (t2+t9)*EL(v3m,1,0)  + (t5+t8)*EL(v3m,2,0)) + EL(v3m,2,0);
    return out;
}


matd_t * matd_quat_multiply(const matd_t * qa, const matd_t * qb)
{
    assert(is_vector_len(qa, 4));
    assert(is_vector_len(qb, 4));

    matd_t * qout = matd_create(4,1);
    EL(qout,0,0) = EL(qa,0,0)*EL(qb,0,0) - EL(qa,1,0)*EL(qb,1,0) - EL(qa,2,0)*EL(qb,2,0) - EL(qa,3,0)*EL(qb,3,0);
    EL(qout,1,0) = EL(qa,0,0)*EL(qb,1,0) + EL(qa,1,0)*EL(qb,0,0) + EL(qa,2,0)*EL(qb,3,0) - EL(qa,3,0)*EL(qb,2,0);
    EL(qout,2,0) = EL(qa,0,0)*EL(qb,2,0) - EL(qa,1,0)*EL(qb,3,0) + EL(qa,2,0)*EL(qb,0,0) + EL(qa,3,0)*EL(qb,1,0);
    EL(qout,3,0) = EL(qa,0,0)*EL(qb,3,0) + EL(qa,1,0)*EL(qb,2,0) - EL(qa,2,0)*EL(qb,1,0) + EL(qa,3,0)*EL(qb,0,0);
    return qout;
}

matd_t * matd_quat_inverse(const matd_t * quat)
{
    double mag = matd_vec_mag(quat);
    matd_t * res = matd_create(4,1);

    res->data[0] = quat->data[0] / mag;
    res->data[1] = -quat->data[1] / mag;
    res->data[2] = -quat->data[2] / mag;
    res->data[3] = -quat->data[3] / mag;

    return res;
}

matd_t * matd_angle_axis_to_quat(double theta, const matd_t * axis)
{
    assert(is_vector_len(axis, 3));

    matd_t * qout  = matd_create(4,1);
    matd_t * axis_norm = matd_vec_normalize(axis);

    EL(qout, 0, 0) = cos(theta/2);
    double s = sin(theta/2);

    EL(qout, 1, 0) = EL(axis_norm, 0, 0) * s;
    EL(qout, 2, 0) = EL(axis_norm, 1, 0) * s;
    EL(qout, 3, 0) = EL(axis_norm, 2, 0) * s;

    // cleanup
    matd_destroy(axis_norm);
    return qout;
}

matd_t * matd_quat_pos_to_matrix(const matd_t *q, const matd_t *pos)
{
    assert(is_vector_len(q, 4));
    assert(is_vector_len(pos, 3));

    // 4 x 4 matrix
    matd_t *m = matd_create(4, 4);
    double w = ELV(q,0), x = ELV(q,1), y = ELV(q,2), z = ELV(q,3);

    EL(m,0,0) = w*w + x*x - y*y - z*z;
    EL(m,0,1) = 2*x*y - 2*w*z;
    EL(m,0,2) = 2*x*z + 2*w*y;

    EL(m,1,0) = 2*x*y + 2*w*z;
    EL(m,1,1) = w*w - x*x + y*y - z*z;
    EL(m,1,2) = 2*y*z - 2*w*x;

    EL(m,2,0) = 2*x*z - 2*w*y;
    EL(m,2,1) = 2*y*z + 2*w*x;
    EL(m,2,2) = w*w - x*x - y*y + z*z;

    if(pos != NULL) {
        EL(m,0,3) = ELV(pos,0);
        EL(m,1,3) = ELV(pos,1);
        EL(m,2,3) = ELV(pos,2);
    }

    EL(m,3,3) = 1;
    return m;
}

matd_t * matd_xyzrpy_to_matrix(const matd_t * xyzrpy)
{
    assert(is_vector_len(xyzrpy, 6));

    double tx = EL(xyzrpy,0,0);
    double ty = EL(xyzrpy,1,0);
    double tz = EL(xyzrpy,2,0);

    double rx = EL(xyzrpy,3,0);
    double ry = EL(xyzrpy,4,0);
    double rz = EL(xyzrpy,5,0);


    double cx = cos(rx), sx = sin(rx);
    double cy = cos(ry), sy = sin(ry);
    double cz = cos(rz), sz = sin(rz);

    // 4 x 4 matrix
    double m[16] = { cy*cz, cz*sx*sy - cx*sz, sx*sz + cx*cz*sy, tx ,
                     cy*sz, cx*cz + sx*sy*sz, cx*sy*sz - cz*sx, ty ,
                     -sy,   cy*sx,            cx*cy,            tz ,
                     0,     0,                0,                1   };


    return matd_create_data(4,4, m);
}

matd_t * matd_xyzrpy_to_xyz(const matd_t * xyzrpy)
{
    matd_t *xyz = matd_create(3,1);
    xyz->data[0] = xyzrpy->data[0];
    xyz->data[1] = xyzrpy->data[1];
    xyz->data[2] = xyzrpy->data[2];
    return xyz;
}

matd_t * matd_xyzrpy_to_rpy(const matd_t * xyzrpy)
{
    matd_t *rpy = matd_create(3,1);
    rpy->data[0] = xyzrpy->data[3];
    rpy->data[1] = xyzrpy->data[4];
    rpy->data[2] = xyzrpy->data[5];
    return rpy;
}

matd_t * matd_xyzrpy_to_quat(const matd_t * xyzrpy)
{
    matd_t *rpy = matd_xyzrpy_to_rpy(xyzrpy);
    matd_t *quat = matd_rpy_to_quat(rpy);
    matd_destroy(rpy);
    return quat;
}

// Reference: "A tutorial on SE(3) transformation parameterizations and
// on-manifold optimization" by Jose-Luis Blanco
matd_t * matd_quat_to_rpy(const matd_t * quat)
{
    assert(is_vector_len(quat, 4));

    matd_t * rpy = matd_create(3,1);

    const double * q = quat->data;
    const double qr = q[0];
    const double qx = q[1];
    const double qy = q[2];
    const double qz = q[3];

    double disc = qr*qy - qx*qz;

    if (fabs(disc+0.5) < DBL_EPSILON) {         // near -1/2
        rpy->data[0] = 0;
        rpy->data[1] = -M_PI/2;
        rpy->data[2] = 2 * atan2(qx, qr);
    }
    else if (fabs(disc-0.5) < DBL_EPSILON) {    // near  1/2
        rpy->data[0] = 0;
        rpy->data[1] = M_PI/2;
        rpy->data[2] = -2 * atan2(qx, qr);
    }
    else {
        // roll
        double roll_a = 2 * (qr*qx + qy*qz);
        double roll_b = 1 - 2 * (qx*qx + qy*qy);
        rpy->data[0] = atan2( roll_a, roll_b );

        // pitch
        rpy->data[1] = asin( 2*disc );

        // yaw
        double yaw_a = 2 * (qr*qz + qx*qy);
        double yaw_b = 1 - 2 * (qy*qy + qz*qz);
        rpy->data[2] = atan2( yaw_a, yaw_b );
    }

    return rpy;
}

matd_t * matd_rpy_to_quat(const matd_t * rpy)
{
    assert(is_vector_len(rpy, 3));

    matd_t * quat = matd_create(4,1);

    double * q = quat->data;

    double roll = rpy->data[0], pitch = rpy->data[1], yaw = rpy->data[2];

    double halfroll = roll / 2;
    double halfpitch = pitch / 2;
    double halfyaw = yaw / 2;

    double sin_r2 = sin( halfroll );
    double sin_p2 = sin( halfpitch );
    double sin_y2 = sin( halfyaw );

    double cos_r2 = cos( halfroll );
    double cos_p2 = cos( halfpitch );
    double cos_y2 = cos( halfyaw );

    q[0] = cos_r2 * cos_p2 * cos_y2 + sin_r2 * sin_p2 * sin_y2;
    q[1] = sin_r2 * cos_p2 * cos_y2 - cos_r2 * sin_p2 * sin_y2;
    q[2] = cos_r2 * sin_p2 * cos_y2 + sin_r2 * cos_p2 * sin_y2;
    q[3] = cos_r2 * cos_p2 * sin_y2 - sin_r2 * sin_p2 * cos_y2;

    return quat;
}

matd_t * matd_matrix_to_quat(const matd_t * mat)
{
    assert(mat->nrows >= 3 && mat->ncols >= 3);


    double T = EL(mat, 0, 0) + EL(mat, 1,1) + EL(mat, 2,2) + 1;
    double S;

    double m0  = EL(mat,0,0), m1 = EL(mat,1,0), m2 = EL(mat,2,0);
    double m4  = EL(mat,0,1), m5 = EL(mat,1,1), m6 = EL(mat,2,1);
    double m8  = EL(mat,0,2), m9 = EL(mat,1,2), m10 = EL(mat,2,2);

    matd_t * un_norm_q = matd_create(4,1);
    double * q = un_norm_q->data;

    if (T > 0.0000001) {
        S = sqrt(T) * 2;
        q[1] = -( m9 - m6 ) / S;
        q[2] = -( m2 - m8 ) / S;
        q[3] = -( m4 - m1 ) / S;
        q[0] = 0.25 * S;
    } else if ( m0 > m5 && m0 > m10 )  {	// Column 0:
        S  = sqrt( 1.0 + m0 - m5 - m10 ) * 2;
        q[1] = -0.25 * S;
        q[2] = -(m4 + m1 ) / S;
        q[3] = -(m2 + m8 ) / S;
        q[0] = (m9 - m6 ) / S;
    } else if ( m5 > m10 ) {			// Column 1:
        S  = sqrt( 1.0 + m5 - m0 - m10 ) * 2;
        q[1] = -(m4 + m1 ) / S;
        q[2] = -0.25 * S;
        q[3] = -(m9 + m6 ) / S;
        q[0] = (m2 - m8 ) / S;
    } else {
        // Column 2:
        S  = sqrt( 1.0 + m10 - m0 - m5 ) * 2;
        q[1] = -(m2 + m8 ) / S;
        q[2] = -(m9 + m6 ) / S;
        q[3] = -0.25 * S;
        q[0] = (m4 - m1 ) / S;
    }

    matd_t * norm_q = matd_vec_normalize(un_norm_q);
    matd_destroy(un_norm_q);
    return norm_q;
}

matd_t * matd_matrix_to_rpy(const matd_t * mat)
{
    matd_t * quat = matd_matrix_to_quat(mat);
    matd_t * rpy = matd_quat_to_rpy(quat);
    matd_destroy(quat);
    return rpy;
}

matd_t * matd_matrix_to_xyz(const matd_t * mat)
{
    assert(mat->ncols == 4 && mat->nrows == 4);
    matd_t *xyz = matd_create(3, 1);
    xyz->data[0] = EL(mat,0,3);
    xyz->data[1] = EL(mat,1,3);
    xyz->data[2] = EL(mat,2,3);
    return xyz;
}

matd_t * matd_matrix_to_xyzrpy(const matd_t * mat)
{
    matd_t * xyzrpy = matd_create(6,1);

    // grab translation
    xyzrpy->data[0] = EL(mat, 0, 3);
    xyzrpy->data[1] = EL(mat, 1, 3);
    xyzrpy->data[2] = EL(mat, 2, 3);

    // grab rotation
    matd_t * rpy = matd_matrix_to_rpy(mat);
    xyzrpy->data[3] = rpy->data[0];
    xyzrpy->data[4] = rpy->data[1];
    xyzrpy->data[5] = rpy->data[2];

    matd_destroy(rpy);
    return xyzrpy;
}

matd_t * matd_matrix_to_xyt(const matd_t * mat)
{
    matd_t * xyt = matd_create(3,1);

    // grab translation
    xyt->data[0] = EL(mat, 0, 3);
    xyt->data[1] = EL(mat, 1, 3);

    // grab rotation
    matd_t * rpy = matd_matrix_to_rpy(mat);
    xyt->data[2] = rpy->data[2];
    matd_destroy(rpy);

    return xyt;
}

matd_t * matd_xyt_to_matrix(const matd_t *xyt)
{
    matd_t *m = matd_identity(4);

    double s = sin(ELV(xyt,2));
    double c = cos(ELV(xyt,2));

    EL(m, 0, 0) = c;
    EL(m, 0, 1) = -s;
    EL(m, 0, 3) = ELV(xyt, 0);

    EL(m, 1, 0) = s;
    EL(m, 1, 1) = c;
    EL(m, 1, 3) = ELV(xyt, 1);
    return m;
}

matd_t *matd_xyt_multiply(const matd_t *a, const matd_t *b)
{
    assert(a->nrows==3 && b->nrows==3);

    double s = sin(MATD_EL(a, 2, 0));
    double c = cos(MATD_EL(a, 2, 0));

    matd_t *x = matd_create(3, 1);

    MATD_EL(x, 0, 0) = c*MATD_EL(b, 0, 0) - s*MATD_EL(b, 1, 0) + MATD_EL(a, 0, 0);
    MATD_EL(x, 1, 0) = s*MATD_EL(b, 0, 0) + c*MATD_EL(b, 1, 0) + MATD_EL(a, 1, 0);
    MATD_EL(x, 2, 0) = MATD_EL(a, 2, 0) + MATD_EL(b, 2, 0);

    return x;
}

// compute:  X = xytMultiply(xytInverse(a), b)
matd_t *matd_xyt_inv_mul31(const matd_t *a, const matd_t *b)
{
    assert(a->nrows==3 && b->nrows==3);

    double theta = MATD_EL(a, 2, 0);
    double ca = cos(theta);
    double sa = sin(theta);
    double dx = MATD_EL(b, 0, 0) - MATD_EL(a, 0, 0);
    double dy = MATD_EL(b, 1, 0) - MATD_EL(a, 1, 0);

    matd_t *x = matd_create(3, 1);

    MATD_EL(x, 0, 0) = ca*dx + sa*dy;
    MATD_EL(x, 1, 0) = -sa*dx + ca*dy;
    MATD_EL(x, 2, 0) = MATD_EL(b, 2, 0) - MATD_EL(a, 2, 0);

    return x;
}


// Compute the inverse of a (x,y,theta) transformation
matd_t *matd_xyt_inverse(const matd_t *a)
{
    assert(a->nrows == 3);

    double s = sin( MATD_EL(a, 2, 0));
    double c = cos( MATD_EL(a, 2, 0));
    matd_t *r = matd_create(3, 1);

    MATD_EL(r, 0, 0) = -s*MATD_EL(a, 1, 0) - c*MATD_EL(a, 0, 0);
    MATD_EL(r, 1, 0) = -c*MATD_EL(a, 1, 0) + s*MATD_EL(a, 0, 0);
    MATD_EL(r, 2, 0) = -MATD_EL(a, 2, 0);

    return r;
}

// Ported from LinAlg
matd_t * matd_quat_slerp(const matd_t * q0, const matd_t * q11, double w)
{
    double dot = matd_vec_dot_product(q0, q11);

    matd_t * q1 = NULL;
    if (dot < 0) {
        // flip sign on one of them so we don't spin the "wrong
        // way" around. This doesn't change the rotation that the
        // quaternion represents.
        dot = -dot;
        q1 = matd_scale(q11, -1);
    } else  {
        q1 = matd_copy(q11);
    }

    // if large dot product (1), slerp will scale both q0 and q1
    // by 0, and normalization will blow up.
    if (dot > 0.95) {

        matd_t *p0 = matd_scale(q0, 1-w);
        matd_t *p1 = matd_scale(q1, w);

        matd_t * q = matd_add(p0, p1);

        matd_destroy(p0);
        matd_destroy(p1);
        matd_destroy(q1);
        return q;
    } else {
        double angle = acos(dot);

        matd_t *p0 = matd_scale(q0, sin(angle*(1-w)));
        matd_t *p1 = matd_scale(q1, sin(angle*w));

        matd_t * q0 = matd_add(p0, p1);

        matd_t * q = matd_vec_normalize(q0);

        matd_destroy(p0);
        matd_destroy(p1);
        matd_destroy(q1);
        matd_destroy(q0);

        return q;
    }
}


/*
matd_t * matd_transform_raw(const matd_t * T, double *p)
{
    matd_t ptmem;
    ptmem.data = p;
    return matd_transform(T, &ptmem);
}
*/

// Applies a 4x4 homogeneous transform T to a 3D point 'point'
// Works correctly for row-based and column-based point vectors
matd_t * matd_transform(const matd_t * T, const matd_t * p)
{
    matd_t * output = matd_create(3,1);
    matd_transform_output(T, p, output);
    return output;
}

zarray_t * matd_transform_zarray(const matd_t * T, const zarray_t * in)
{
    zarray_t *out = zarray_create(sizeof(matd_t*));

    for (int i = 0; i < zarray_size(in); i++)
    {
        matd_t *p_in = NULL;
        zarray_get(in, i, &p_in);

        matd_t *p_out = matd_transform(T, p_in);
        zarray_add(out, &p_out);
    }

    return out;
}

// Applies a 4x4 homogeneous transform T to a 3D point 'point'
// Works correctly for row-based and column-based point vectors
void matd_transform_output(const matd_t * T, const matd_t * p, matd_t * o)
{
    o->data[0] =  EL(T,0,0)*ELV(p,0) + EL(T,0,1)*ELV(p,1) + EL(T,0,2)*ELV(p,2) + EL(T,0,3);
    o->data[1] =  EL(T,1,0)*ELV(p,0) + EL(T,1,1)*ELV(p,1) + EL(T,1,2)*ELV(p,2) + EL(T,1,3);
    o->data[2] =  EL(T,2,0)*ELV(p,0) + EL(T,2,1)*ELV(p,1) + EL(T,2,2)*ELV(p,2) + EL(T,2,3);

}

matd_t * matd_translate3(double x, double y, double z)
{
    matd_t * obj = matd_identity(4);
    double * mat44 = obj->data;
    mat44[0*4 + 3] = x;
    mat44[1*4 + 3] = y;
    mat44[2*4 + 3] = z;
    return obj;
}

matd_t * matd_rotate_x(double theta)
{
    matd_t * obj = matd_identity(4);
    double * mat44 = obj->data;
    double s = sin(theta);
    double c = cos(theta);
    mat44[0*4 + 0] =  1.0;
    mat44[1*4 + 1] =  c;
    mat44[1*4 + 2] =  -s;
    mat44[2*4 + 1] =  s;
    mat44[2*4 + 2] =  c;
    return obj;
}

matd_t * matd_rotate_y(double theta)
{
    matd_t * obj = matd_identity(4);
    double * mat44 = obj->data;
    double s = sin(theta);
    double c = cos(theta);
    mat44[0*4 + 0] =  c;
    mat44[0*4 + 2] =  s;
    mat44[1*4 + 1] =  1.0;
    mat44[2*4 + 0] =  -s;
    mat44[2*4 + 2] =  c;
    return obj;
}

matd_t * matd_rotate_z(double theta)
{
    matd_t * obj = matd_identity(4);
    double * mat44 = obj->data;
    double s = sin(theta);
    double c = cos(theta);
    mat44[0*4 + 0] =  c;
    mat44[0*4 + 1] = -s;
    mat44[1*4 + 0] =  s;
    mat44[1*4 + 1] =  c;
    mat44[2*4 + 2] =  1.0;
    return obj;
}

matd_t * matd_scale3(double scale)
{
    matd_t * obj = matd_create(4,4);
    double * mat44 = obj->data;
    mat44[0*4 + 0] =  scale;
    mat44[1*4 + 1] =  scale;
    mat44[2*4 + 2] =  scale;
    mat44[3*4 + 3] =  1.0;
    return obj;
}
