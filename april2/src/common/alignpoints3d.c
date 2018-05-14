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

#include "alignpoints3d.h"
#include "matd.h"

alignpoints3d_t *alignpoints3d_create()
{
    alignpoints3d_t *ap = calloc(1, sizeof(alignpoints3d_t));
    ap->as = zarray_create(sizeof(double[3]));
    ap->bs = zarray_create(sizeof(double[3]));
    ap->ws = zarray_create(sizeof(double));

    return ap;
}

void alignpoints3d_destroy(alignpoints3d_t *ap)
{
    if (!ap)
        return;

    zarray_destroy(ap->as);
    zarray_destroy(ap->bs);
    zarray_destroy(ap->ws);
    free(ap);
}

void alignpoints3d_add_weighted(alignpoints3d_t *ap, const double *a, const double *b, double weight)
{
    assert(weight==1); // others not really implemented
    zarray_add(ap->as, a);
    zarray_add(ap->bs, b);
    zarray_add(ap->ws, &weight);
}

void alignpoints3d_add(alignpoints3d_t *ap, const double *a, const double *b)
{
    alignpoints3d_add_weighted(ap, a, b, 1);
}

void alignpoints3d_compute(alignpoints3d_t *ap, double *M)
{
    // returns T, an n+1 by n+1 homogeneous transform points of dimension n
    // from list a to list b using method described in
    // "Least Squares Estimation of Transformation Parameters Between Two Point Patterns"
    // by Shinji Umeyana
    //
    // Algorithm overiew:
    //   a. Compute centroids of both lists, and center a, b at origin
    //   b. compute M[n][n] = \Sum b_i * a_i^t
    //   c. given M = UDV^t via singular value decomposition, compute rotation
    //      via R = USV^t where S = diag(1,1 .. 1, det(U)*det(V));
    //   d. result computed by compounding differences in centroid and rotation matrix
    //

    // note mean[3] will be 1 for homogeneous transform later.
    double mean_a[4] = { 0, 0, 0, 1 }, mean_b[4] = { 0, 0, 0, 1}, weight = 0;

    assert(zarray_size(ap->as) == zarray_size(ap->bs));
    assert(zarray_size(ap->as) == zarray_size(ap->ws));

    for (int i = 0; i < zarray_size(ap->as); i++) {
        double *a, *b;
        zarray_get_volatile(ap->as, i, &a);
        zarray_get_volatile(ap->bs, i, &b);

        double w;
        zarray_get(ap->ws, i, &w);

        for (int j = 0; j < 3; j++) {
            mean_a[j] += a[j] * w;
            mean_b[j] += b[j] * w;
        }

        weight += w;
    }

    for (int i = 0; i < 3; i++) {
        mean_a[i] /= weight;
        mean_b[i] /= weight;
    }

    matd_t *C = matd_create(4, 4);

    for (int i = 0; i < zarray_size(ap->as); i++) {
        double *a, *b;
        zarray_get_volatile(ap->as, i, &a);
        zarray_get_volatile(ap->bs, i, &b);

        double w;
        zarray_get(ap->ws, i, &w);

        // XXX where does W go here?
        for (int row = 0; row < 3; row++)
            for (int col = 0; col < 3; col++)
                MATD_EL(C, row, col) += w * (b[row] - mean_b[row]) * (a[col] - mean_a[col]);
    }

    // scale for numerical precision? (dubious)
    matd_svd_t svd = matd_svd(C);

    matd_t *S = matd_identity(4);
    MATD_EL(S, 2, 2) = matd_det(svd.U) * matd_det(svd.V) < 0 ? -1 : 1;

    matd_t *R = matd_op("M*M*M'", svd.U, S, svd.V);

    matd_print(R, "%15f");

    matd_t *Ma = matd_create_data(4, 1, mean_a);
    matd_t *Mb = matd_create_data(4, 1, mean_b);
    matd_t *T = matd_op("M-M*M", Mb, R, Ma);

    memset(M, 0, 16*sizeof(double));

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            M[4*i+j] = MATD_EL(R, i, j);
    for (int i = 0; i < 3; i++)
        M[4*i+3] = MATD_EL(T, i, 0);

    M[15] = 1;

    matd_destroy(svd.U);
    matd_destroy(svd.S);
    matd_destroy(svd.V);

    matd_destroy(S);
    matd_destroy(C);
    matd_destroy(Ma);
    matd_destroy(Mb);
    matd_destroy(T);
}

void alignpoints3d_reset(alignpoints3d_t *ap)
{
    zarray_clear(ap->as);
    zarray_clear(ap->bs);
    zarray_clear(ap->ws);
}
