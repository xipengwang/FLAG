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
#include <stdlib.h>
#include <assert.h>
#include <stdint.h>
#include <math.h>

#include "../matd.h"

static int randi()
{
    int v = random()&31;
    v -= 15;
    return v;
}

int is_ident(matd_t *A)
{
    double eps = 0.000001;

    for (int i = 0; i < A->nrows; i++) {
        for (int j = 0; j < A->ncols; j++) {

            double v = MATD_EL(A, i, j);
            if (i == j)
                v -= 1;

            double err = fabs(v);

            if (err > eps)
                return 0;
        }
    }
    return 1;
}


int main(int argc, char *argv[])
{
    int debug = 0;

    for (int iter = 0; iter < 10000; iter++) {
        srandom(iter);

        if (debug)
            printf("iter: %d\r", iter);

        // create a matrix and try to make sure that it is
        // non-singular.
        int sz = (random()&15) + 1;
        matd_t *A = matd_create(sz, sz);

        for (int i = 0; i < A->nrows; i++)
            for (int j = 0; j < A->ncols; j++)
                MATD_EL(A, i, j) = randi();

        matd_svd_t svd = matd_svd(A);
        double mineig = MATD_EL(svd.S, svd.S->nrows-1, svd.S->ncols - 1);

        if (fabs(mineig) < 1E-4) {
            goto cleanup;
        }

        matd_t *inv = matd_inverse(A);

        if (debug) {
            printf("inv1: \n");
            matd_print(matd_op("M*M", A, inv), "%10f");
        }

        if (!is_ident(matd_op("M*M", A, inv))) {
            printf("FAIL iter %d, sz = %d, mineig = %15f\n", iter, sz, mineig);
            printf("A:\n");
            matd_print(A, "%15f");
            printf("inv:\n");
            matd_print(inv, "%15f");
            printf("A*inv(A)\n");
            matd_print(matd_op("M*M", A, inv), "%15f");
            printf("S:\n");
            matd_print(svd.S, "%15f");
            exit(-1);
        }

      cleanup:
        matd_destroy(svd.U);
        matd_destroy(svd.S);
        matd_destroy(svd.V);

        matd_destroy(A);
    }
}
