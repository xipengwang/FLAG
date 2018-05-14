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

int main(int argc, char *argv[])

{
    int niter = 100000;
    if (argc > 1)
        niter = atoi(argv[1]);

    matd_t **special_matrices = (matd_t*[]) { matd_identity(1),
                                              matd_identity(2),
                                              matd_identity(3),
                                              matd_identity(4),
                                              matd_create(1, 1),
                                              matd_create(2, 2),
                                              matd_create(5, 5),
                                              matd_create_data(2, 2, (double[]) { 7, 0, 0, 91.4 }),
                                              matd_create_data(2, 2, (double[]) { 0, 0, 5, 0 }),
                                              NULL
    };

    int special_matrices_done = 0;

    for (int iter = 0; iter < niter; iter++) {
        srandom(iter);

        if ((iter % 10000) == 0) {
            printf("iter: %d\r", iter);
            fflush(NULL);
        }

        int nrows, ncols;
        matd_t *A;

        if (!special_matrices_done) {
            A = special_matrices[iter];

            nrows = A->nrows;
            ncols = A->ncols;

            if (special_matrices[iter+1] == NULL)
                special_matrices_done = 1;
        } else {
            // create a random matrix
            nrows = (random()&7) + 1;
            ncols = nrows;

            A = matd_create(nrows, ncols);

            for (int i = 0; i < A->nrows; i++)
                for (int j = 0; j < A->ncols; j++)
                    MATD_EL(A, i, j) = randi();

        }

        matd_plu_t *mlu = matd_plu(A);

        matd_t *P = matd_plu_p(mlu), *L = matd_plu_l(mlu), *U = matd_plu_u(mlu);

        matd_t *PLU = matd_op("M*M*M", P, L, U);

        char *bad = NULL;

        double eps = 1e-7;

        // ensure PLU == A.
        for (int i = 0; i < nrows; i++) {
            for (int j = 0; j < ncols; j++) {
                if (fabs(MATD_EL(A, i, j) - MATD_EL(PLU, i, j)) > eps)
                    bad = "PLU != A";
            }
        }

        // ensure L is lower triangular
        for (int i = 0; i < nrows; i++) {
            if (fabs(MATD_EL(L, i, i) - 1) > eps)
                bad = "L not 1 along diagonal";

            for (int j = i+1; j < nrows; j++) {
                if (fabs(MATD_EL(L, i, j) > eps))
                    bad = "L not lower diagonal";
            }
        }


        // ensure U is upper triangular
        for (int i = 0; i < nrows; i++) {
            for (int j = 0; j < i; j++) {
                if (fabs(MATD_EL(U, i, j) > eps))
                    bad = "U not upper";
            }
        }

        if (bad) {
            printf("failure: %s\n", bad);

            printf("A: \n");
            matd_print(A, "%15f");

            printf("P: \n");
            matd_print(P, "%15f");
            printf("L: \n");
            matd_print(L, "%15f");
            printf("U: \n");
            matd_print(U, "%15f");

            printf("PLU: \n");
            matd_print(PLU, "%15f");

            exit(-1);
        }

        matd_destroy(A);
        matd_destroy(P);
        matd_destroy(L);
        matd_destroy(U);
        matd_destroy(PLU);
    }
}
