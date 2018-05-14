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

#ifndef MATD_COORDS_H
#define MATD_COORDS_H

#include "common/matd.h"
#include "common/zarray.h"

// quaternion vector format: [ w, x, y, z ]
// Note: 'w' sometimes referred to as 'r'

#ifdef __cplusplus
extern "C" {
#endif

matd_t * matd_quat_rotate(const matd_t * q, const matd_t * in3);
matd_t * matd_quat_multiply(const matd_t * qa, const matd_t * qb);
matd_t * matd_quat_inverse(const matd_t * quat);

matd_t * matd_angle_axis_to_quat(double angle, const matd_t * axis3);

matd_t * matd_quat_pos_to_matrix(const matd_t *q, const matd_t *pos);
matd_t * matd_xyzrpy_to_matrix(const matd_t * xyzrpy);

matd_t * matd_xyzrpy_to_xyz(const matd_t * xyzrpy);
matd_t * matd_xyzrpy_to_rpy(const matd_t * xyzrpy);
matd_t * matd_xyzrpy_to_quat(const matd_t * xyzrpy);

// These methods interpret the top left 3x3 matrix
// of 'mat' as a rotation matrix, and return the corresponding
// summary (quat or rpy) as a 4 or 3 vector, respectively
matd_t * matd_matrix_to_quat(const matd_t * mat);
matd_t * matd_matrix_to_rpy(const matd_t * mat);

// clip the translation out of the 4x4 homogeneous RBT
matd_t * matd_matrix_to_xyz(const matd_t * mat);

matd_t * matd_matrix_to_xyzrpy(const matd_t * mat);

matd_t * matd_matrix_to_xyt(const matd_t * mat);
matd_t * matd_xyt_to_matrix(const matd_t *xyt);

/**
 * Multiply two (x, y, theata) 2D RBTs
 * It is the caller's responsibility to call matd_destroy() on the returned
 * matrix.
 */
matd_t *matd_xyt_multiply(const matd_t *a, const matd_t *b);

/**
 * Computes x = xyt_multiply(xyt_inverse(a), b) where a and b are 3x1 vectors.
 * It is the caller's responsibility to call matd_destroy() on the returned
 * matrix.
 **/
matd_t *matd_xyt_inv_mul31(const matd_t *a, const matd_t *b);

/**
 * Computes the inverse of a (x, y, theta) transformation. It is the caller's
 * responsibility to call matd_destroy() on the returned matrix.
 **/
matd_t *matd_xyt_inverse(const matd_t *a);


// Interpolate quaternions from q0 (w=0) to q1 (w=1).
matd_t * matd_quat_slerp(const matd_t * q0, const matd_t * q1, double w);


matd_t * matd_quat_to_rpy(const matd_t * quat);
matd_t * matd_rpy_to_quat(const matd_t * rpy);

// Applies a 4x4 homogeneous transform T to a 3D point 'point'
// In either data format, point should contain 3-elements
matd_t * matd_transform_raw(const matd_t * T, double *point);
matd_t * matd_transform(const matd_t * T, const matd_t * point);
zarray_t * matd_transform_zarray(const matd_t * T, const zarray_t * points);

// Same as above, but allows user to specify output memory
void matd_transform_output(const matd_t * T, const matd_t * point, matd_t * output3);

// These functions return 4x4 homogeneous transform matrices
matd_t * matd_translate3(double x, double y, double z);
matd_t * matd_rotate_x(double theta);
matd_t * matd_rotate_y(double theta);
matd_t * matd_rotate_z(double theta);
matd_t * matd_scale3(double scale_xyz);

#ifdef __cplusplus
}
#endif

#endif
