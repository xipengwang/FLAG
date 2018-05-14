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

#ifndef _CAMERA_MATH_H
#define _CAMERA_MATH_H

#include "common/matd.h"
#include "common/zarray.h"

matd_t   *pinhole_transform(const matd_t *T, const matd_t *p);
matd_t   *point_transform(const matd_t *T, const matd_t *p);

/**
 * Snap a vector onto the z==1 plane, or unit magnitude sphere
 */
matd_t   *snap_ray_to_plane(const matd_t *p);
matd_t   *snap_ray_to_sphere(const matd_t *p);

/**
 * Compute the K*B2C product for the camera matrix P
 */
matd_t   *make_camera_matrix(const matd_t *K, const matd_t *B2C);

/**
 * Project a set of 3D points using 3x3 intrinsics matrix K and
 * 4x4 homogeneous rigid-body transformation B2C (body-to-camera)
 */
zarray_t *project_intrin_list(const matd_t *K, const matd_t *B2C,
                              zarray_t *xyzs);

matd_t   *project_intrin(const matd_t *K, const matd_t *B2C,
                         const matd_t *xyz);

/**
 * Compute a transformation for plotting images and, for example, feature
 * detections on top of them
 */
matd_t   *image_to_vis_transform(int imwidth, int imheight,
                                 double XY0[2], double XY1[2], int _flip);

/**
 * Estimate a 3x3 homography from a set of 2D point correspondences.
 * A minimum of 4 correspondences are required.
 */
matd_t *camera_math_estimate_homography(const zarray_t *xys,
                                        const zarray_t *xy_primes);

/**
 * Compute the transformation that translates the centroid to the origin
 * and scales the RMS distance to the origin to sqrt(2). Used for computing
 * homographies and fundamental matrices
 */
matd_t *camera_math_compute_normalizing_transform(const zarray_t *xys);

/**
 * Decompose a 3x3 homography using the 3x3 camera intrinsics matrix K. By
 * convention, Z is assumed to point *into* the image. The point p is a point
 * in the homography world frame that must appear in front of the camera, such
 * as a point on an apriltag or apriltag mosaic.
 */
matd_t *camera_math_decompose_homography(const matd_t *H, const matd_t *K,
                                         const matd_t *p);


matd_t * c_triangulate(matd_t* Pleft, matd_t* Pright, double pxleft[2], double pxright[2]);

matd_t * camera_math_triangulate(matd_t * Pleft, matd_t *xy_left,
                                 matd_t * Pright, matd_t *xy_right);

matd_t * camera_math_triangulate_iter(matd_t * Pleft, matd_t *xy_left,
                                      matd_t * Pright, matd_t *xy_right);

#endif
