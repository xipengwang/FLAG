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
    double eps = 1E-7;

    int niter = 100000;
    if (argc > 1)
        niter = atoi(argv[1]);

    matd_t **special_matrices = (matd_t*[]) { matd_identity(1),
                                              matd_identity(2),
                                              matd_identity(3),
                                              matd_identity(4),
                                              matd_create(1, 1),
                                              matd_create(2, 1),
                                              matd_create(1, 2),
                                              matd_create(2, 2),
                                              matd_create(5, 2),
                                              matd_create(2, 5),
                                              matd_create_data(2, 2, (double[]) { 7, 0, 0, 91.4 }),
                                              matd_create_data(4, 5, (double[]){ 1, 0, 0, 0, 2,
                                                          0, 0, 3, 0, 0,
                                                          0, 0, 0, 0, 0,
                                                          0, 4, 0, 0, 0,
                                                          }),

                                              matd_create_data(2, 3, (double[]) { 0.998750260395, 0.049979169271, -0.499791692707,
                                                          0.988771077936, 0.149438132474, -1.494381324736 }),

                                              matd_create_data(9,9, (double[]) {
                                                      0.4208930,   0.8222886,   0.9709885,   0.9634991,   0.5123640,   0.6404429,   0.3059198,   0.4866123,   0.6863641,
                                                          0.5698303,   0.0056370,   0.4480595,   0.9491955,   0.8944775,   0.3988927,   0.0261708,   0.8891022,   0.0243235,
                                                          0.6672190,   0.7762727,   0.7153878,   0.5932580,   0.0924744,   0.9473060,   0.7867667,   0.1289006,   0.8077791,
                                                          0.9840936,   0.2542941,   0.6284734,   0.9881202,   0.1422128,   0.7842264,   0.8584235,   0.6614492,   0.8433338,
                                                          0.2127137,   0.1170212,   0.5789272,   0.9867476,   0.9532242,   0.5656623,   0.8578596,   0.4667560,   0.5226499,
                                                          0.1991618,   0.5929621,   0.2457745,   0.0586837,   0.4911475,   0.4323033,   0.9337985,   0.7959346,   0.2908861,
                                                          0.5309674,   0.3844642,   0.6883988,   0.9547806,   0.4658898,   0.4763155,   0.0785583,   0.5050301,   0.4564309,
                                                          0.1477745,   0.5204891,   0.0531454,   0.9631223,   0.6943927,   0.1554204,   0.5900272,   0.1283087,   0.0854261,
                                                          0.1669857,   0.1902987,   0.5722845,   0.3435725,   0.5796787,   0.4537727,   0.5549823,   0.6845101,   0.8245620 }),


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
            nrows = (random()&31) + 1;
            ncols = (random()&31) + 1;

            A = matd_create(nrows, ncols);

            for (int i = 0; i < A->nrows; i++)
                for (int j = 0; j < A->ncols; j++)
                    MATD_EL(A, i, j) = randi();

        }


        matd_svd_t svd = matd_svd(A);

        matd_t *USVT = matd_op("M*M*M'", svd.U, svd.S, svd.V);

        // check dimensions
        assert(USVT->nrows == A->nrows);
        assert(USVT->ncols == A->ncols);
        assert(svd.U->nrows == svd.U->ncols && svd.U->nrows == A->nrows);
        assert(svd.V->nrows == svd.V->ncols && svd.V->nrows == A->ncols);
        assert(svd.S->nrows == A->nrows && svd.S->ncols == A->ncols);

        // no NANs anywhere
        for (int i = 0; i < svd.U->nrows; i++) {
            for (int j = 0; j < svd.U->ncols; j++) {
                if (!isfinite(MATD_EL(svd.U, i, j)))
                    goto bad;
            }
        }

        for (int i = 0; i < svd.V->nrows; i++) {
            for (int j = 0; j < svd.V->ncols; j++) {
                if (!isfinite(MATD_EL(svd.V, i, j)))
                    goto bad;
            }
        }

        for (int i = 0; i < svd.S->nrows; i++) {
            for (int j = 0; j < svd.S->ncols; j++) {
                if (!isfinite(MATD_EL(svd.S, i, j)))
                    goto bad;
            }
        }

        // does USV' == A?
        for (int i = 0; i < A->nrows; i++) {
            for (int j = 0; j < A->ncols; j++) {
                double err = MATD_EL(A, i, j) - MATD_EL(USVT, i, j);
                if (fabs(err) > eps) {
                    printf("USV' != A\n");
                    goto bad;
                }
            }
        }

        // is S diagonal, sorted and positive?
        double lasteig = HUGE;

        for (int i = 0; i < svd.S->nrows; i++) {
            for (int j = 0; j < svd.S->ncols; j++) {
                double v = MATD_EL(svd.S, i, j);
                if (i != j) {
                    if (fabs(v) > eps) {
                        printf("S not diagonal\n");
                        goto bad;
                    }
                    continue;
                }

                if (v > lasteig) {
                    printf("singular values not sorted\n");
                    goto bad;
                }

                if (v < 0) {
                    printf("singular value negative\n");
                    goto bad;
                }

                lasteig = v;
            }
        }

        // is U/V unitary?
        for (int loop = 0; loop < 2; loop++) {

            // test both U and V using the same code below.
            matd_t *M = (loop == 0) ? svd.U : svd.V;

            // check rows for unit magnitude
            for (int rowi = 0; rowi < M->nrows; rowi++) {
                double mag2 = 0;
                for (int col = 0; col < M->ncols; col++) {
                    mag2 += MATD_EL(M, rowi, col) * MATD_EL(M, rowi, col);
                }
                if (fabs(sqrt(mag2)-1) > 1E-5) {
                    printf("Row %d not magnitude 1, mag %.15f\n", rowi, mag2);
                    goto bad;
                }
            }

            // check cols for unit magnitude
            for (int coli = 0; coli < M->ncols; coli++) {
                double mag2 = 0;
                for (int row = 0; row < M->nrows; row++) {
                    mag2 += MATD_EL(M, row, coli) * MATD_EL(M, row, coli);
                }

                if (fabs(sqrt(mag2)-1) > 1E-5) {
                    printf("Col %d not magnitude 1, mag %.15f\n", coli, sqrt(mag2));
                    goto bad;
                }
            }

            // check rows for ortho-normality
            for (int rowi = 0; rowi < M->nrows; rowi++) {
                for (int rowj = rowi+1; rowj < M->nrows; rowj++) {
                    double dot = 0;
                    for (int col = 0; col < M->ncols; col++) {
                        dot += MATD_EL(M, rowi, col) * MATD_EL(M, rowj, col);
                    }
                    if (fabs(dot) > 1E-6) {
                        printf("Row %d %d not orthogonal; dot=%.15f\n", rowi, rowj, dot);
                        goto bad;
                    }
                }
            }

            // check columns for ortho-normality
            for (int coli = 0; coli < M->ncols; coli++) {
                for (int colj = coli+1; colj < M->ncols; colj++) {
                    double dot = 0;
                    for (int row = 0; row < M->nrows; row++) {
                        dot += MATD_EL(M, row, coli) * MATD_EL(M, row, colj);
                    }
                    if (fabs(dot) > 1E-6) {
                        printf("Cols %d %d not orthogonal; dot=%.15f\n", coli, colj, dot);
                        goto bad;
                    }
                }
            }

        }

//        if (iter == 12)
//            goto bad;

        matd_destroy(svd.U);
        matd_destroy(svd.S);
        matd_destroy(svd.V);

        matd_destroy(A);
        matd_destroy(USVT);
        continue;

      bad:
        printf("iter %d\n\n", iter);

        printf("A:\n");
        matd_print(A, "%15f");
        printf("U:\n");
        matd_print(svd.U, "%15f");
        printf("S:\n");
        matd_print(svd.S, "%15f");
        printf("V:\n");
        matd_print(svd.V, "%15f");
        printf("USV':\n");
        matd_print(USVT, "%15f");
        exit(-1);
    }
}
