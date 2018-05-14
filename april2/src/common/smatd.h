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

/*$LICENSE*/

#ifndef _SMATD_H
#define _SMATD_H

// sparse matrix library.  ebolson 9/2013, adapted from
// april.jmat. Unlike jmat, doesn't support per-row variation of
// sparse/dense representation. However, API supports some specific
// dense operations with matd objects.

typedef struct
{
    int length; // logical length
    int alloc;  // allocated size of indices and values.

    int nz;     // non-zero length (and valid length of indices/values arrays)
    int *indices;
    double *values;

    int getcursor, setcursor;
} svecd_t;

typedef struct
{
    int nrows;
    int ncols;

    svecd_t *rows;
} smatd_t;

#define TYPE double

smatd_t *smatd_create(int nrows, int ncols);
smatd_t *smatd_create_data(int nrows, int ncols, const TYPE *data);
smatd_t *smatd_identity(int nrows, int ncols);
void smatd_destroy(smatd_t *m);
void smatd_set(smatd_t *m, int row, int col, TYPE val);
TYPE smatd_get(smatd_t *m, int row, int col);

void smatd_print(smatd_t *m, const char *fmt);

int smatd_nz(smatd_t *A);

// CAUTION: Returns the internal representation of this
// matrix. Modifications of this vector will affect the matrix. Do not
// attempt to destroy the underlying vector.
svecd_t *smatd_get_row_volatile(smatd_t *m, int row);

// unlike get_row, this returns a copy of the underlying data. This
// vector must be destroyed by the caller.
svecd_t *smatd_copy_column(smatd_t *m, int column);

smatd_t *smatd_multiply(smatd_t *a, smatd_t *b);
smatd_t *smatd_upper_right(smatd_t *a);
smatd_t *smatd_transpose(smatd_t *a);

typedef struct
{
    smatd_t *u;
    int is_spd;

} smatd_chol_t;

typedef struct
{
    smatd_t *ldu;
} smatd_ldu_t;

// Compute a Cholesky factorization. No pivoting is performed
// (deliberately--- so that we do not undermine an externally-applied
// sparsity-maximizing variable ordering. The matrix must be square and
// SPD, or the results are undefined.
smatd_chol_t *smatd_chol(smatd_t *a);
void smatd_chol_solve(smatd_chol_t *chol, const TYPE *b, TYPE *x);
void smatd_chol_destroy(smatd_chol_t *chol);

// Compute an LDU factorization. No pivoting is performed
// (deliberately--- so that we do not undermine an externally-applied
// sparsity-maximizing variable ordering. The matrix can be any shape,
// and should have rank equal to its minimum dimension (e.g., a 10x3
// matrix should have rank 3.)
smatd_ldu_t *smatd_ldu(smatd_t *a);

// get the L, D, and U matrices computed by smatd_ldu. Note: not
// needed to solve systems.
// LDU exists for any matrix. L: MxR, D: RxR, U: RxN
// Where R is min(M,R)
void smatd_ldu_get(smatd_ldu_t *sldu, smatd_t **l_out, smatd_t **d_out, smatd_t **u_out);

void smatd_ldu_destroy(smatd_ldu_t *sldu);

#endif
