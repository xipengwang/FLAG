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

// sparse mat
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>

#include "assert.h"
//#include "timeprofile.h"

#include "smatd.h"

// make future smatf/smati implementations easier.
#define TYPE double

#define SVEC_MIN_CAPACITY 16

/////////////////////////////////////////////////
// vec operations

svecd_t *svecd_create(int length)
{
    svecd_t *v = calloc(1, sizeof(svecd_t));
    v->length = length;

    return v;
}

// beware: smatd allocates svecd_ts directly. Do not call
void svecd_destroy(svecd_t *v)
{
    if (!v)
        return;

    free(v->indices);
    free(v->values);

    // this is why you can't call svecd_destroy on a svec allocated by a smatd
    free(v);
}

void svecd_ensure_capacity(svecd_t *v, int mincap)
{
    if (v->alloc >= mincap)
        return;

    int newcap = v->alloc;
    if (newcap < SVEC_MIN_CAPACITY)
        newcap = SVEC_MIN_CAPACITY;

    while (newcap < mincap) {
        newcap *= 2;
    }

    v->indices = realloc(v->indices, sizeof(int)*newcap);
    v->values = realloc(v->values, sizeof(TYPE)*newcap);
    v->alloc = newcap;
}

// make the ith element correspond to the (idx, v) tuple, moving
// anything after it as necessary.l Position 'i' must be the correct
// position for idx. 'idx' must not already be in the vec.
void svecd_insert(svecd_t *v, int i, int idx, TYPE val)
{
    if (val==0)
        return;

    svecd_ensure_capacity(v, v->nz + 1);

    memmove(&v->indices[i+1], &v->indices[i], sizeof(int)*(v->nz - i));
    memmove(&v->values[i+1], &v->values[i], sizeof(TYPE)*(v->nz - i));

    v->indices[i] = idx;
    v->values[i] = val;
    v->nz++;
}

void svecd_set(svecd_t *v, int idx, TYPE val)
{
    if (v->setcursor < 0 || v->setcursor >= v->nz)
        v->setcursor = v->nz / 2;

    if (v->nz == 0) {
        if (val == 0)
            return;

        svecd_ensure_capacity(v, 1);

        v->indices[0] = idx;
        v->values[0] = val;
        v->nz = 1;
        return;
    }

    if (v->indices[v->setcursor] == idx) {
        v->values[v->setcursor] = val;
        return;
    }

    // search.
    if (v->indices[v->setcursor] < idx) {
        // search up
        while (v->setcursor+1 < v->nz && v->indices[v->setcursor+1] <= idx)
            v->setcursor++;

        if (v->indices[v->setcursor] == idx) {
            v->values[v->setcursor] = val;
            return;
        }

        svecd_insert(v, v->setcursor+1, idx, val);
        return;
    } else {
        // search down
        while (v->setcursor-1 >= 0 && v->indices[v->setcursor-1] >= idx)
            v->setcursor--;

        if (v->indices[v->setcursor] == idx) {
            v->values[v->setcursor] = val;
            return;
        }

        svecd_insert(v, v->setcursor, idx, val);
        return;
    }
}

TYPE svecd_get(svecd_t *v, int idx)
{
    if (v->nz == 0)
        return 0;

    if (v->getcursor >= v->nz || v->getcursor < 0)
        v->getcursor = v->nz / 2;

    if (v->indices[v->getcursor] < idx) {
        // search up
        while (v->getcursor+1 < v->nz && v->indices[v->getcursor+1] <= idx)
            v->getcursor++;

        if (v->indices[v->getcursor]==idx)
            return v->values[v->getcursor];

        return 0;
    } else {
        // search down
        while (v->getcursor-1 >= 0 && v->indices[v->getcursor-1] >= idx)
            v->getcursor--;

        if (v->indices[v->getcursor] == idx)
            return v->values[v->getcursor];

        return 0;
    }

    assert(0);
}

TYPE svecd_dot_product(svecd_t *a, svecd_t *b)
{
    if (a->nz == 0 || b->nz == 0)
        return 0;

    int aidx = 0, bidx = 0;
    int ai = a->indices[aidx], bi = b->indices[bidx];

    TYPE acc = 0;

    while (aidx < a->nz && bidx < b->nz) {

        if (ai == bi) {
            acc += a->values[aidx]*b->values[bidx];
            aidx++;
            bidx++;
            ai = a->indices[aidx];
            bi = b->indices[bidx];
            continue;
        }

        if (ai < bi) {
            aidx++;
            ai = a->indices[aidx];
        } else {
            bidx++;
            bi = b->indices[bidx];
        }
    }

    return acc;
}

// v = v * s
void svecd_scale(svecd_t *v, double scale)
{
    for (int i = 0; i < v->nz; i++) {
        v->values[i] *= scale;
    }
}

// beginning at element i0, scale the element
void svecd_scale_i0(svecd_t *v, double scale, int i0)
{
    for (int i = 0; i < v->nz; i++) {
        if (v->indices[i] >= i0)
            v->values[i] *= scale;
    }
}

void svecd_print(svecd_t *a, const char *fmt)
{
    for (int i = 0; i < a->length; i++)
        printf(fmt, svecd_get(a, i));
    printf("\n");
}


// Add vectors 'a' and 'b', using only portions of each vector. Aidx
// and bidx give the first index into a->indices and b->indices.

 void svecd_add_i0_x(svecd_t *x, svecd_t *a, svecd_t *b, double bscale, int aidx, int bidx)
{
    int maxsz = a->nz - aidx + b->nz - bidx;

    // XXX beware of stack overflow
    TYPE tmp_values[maxsz];
    int tmp_indices[maxsz];
    int tmp_nz = 0;

    int ai = (aidx < a->nz) ? a->indices[aidx] : INT32_MAX;
    int bi = (bidx < b->nz) ? b->indices[bidx] : INT32_MAX;

    while (aidx < a->nz || bidx < b->nz) {
        if (ai == bi) {
            // add and increment both.
            tmp_values[tmp_nz] = a->values[aidx] + b->values[bidx] * bscale;
            tmp_indices[tmp_nz] = ai;
            tmp_nz++;
            aidx++;
            ai = (aidx < a->nz) ? a->indices[aidx] : INT32_MAX;
            bidx++;
            bi = (bidx < b->nz) ? b->indices[bidx] : INT32_MAX;
        } else if (ai < bi) {
            tmp_values[tmp_nz] = a->values[aidx];
            tmp_indices[tmp_nz] = ai;
            tmp_nz++;
            aidx++;
            ai = (aidx < a->nz) ? a->indices[aidx] : INT32_MAX;
        } else {
            // bi < ai
            tmp_values[tmp_nz] = b->values[bidx] * bscale;
            tmp_indices[tmp_nz] = bi;
            tmp_nz++;
            bidx++;
            bi = (bidx < b->nz) ? b->indices[bidx] : INT32_MAX;
        }
    }

    svecd_ensure_capacity(x, tmp_nz);

    memcpy(x->indices, tmp_indices, sizeof(int)*tmp_nz);
    memcpy(x->values, tmp_values, sizeof(TYPE)*tmp_nz);
    x->nz = tmp_nz;
}

// Add vectors 'a' and 'b', treating elements in 'a' before index a0
// as zero, and elements in 'b' before index b0 as zero. Elements in b
// are scaled by 'bscale'.  In-place modifications are allowed (i.e.,
// x may be equal to a or b).
void svecd_add_i0(svecd_t *x, const svecd_t *a, const svecd_t *b, double bscale, int a0, int b0)
{
    int aidx = 0, bidx = 0;

    while (aidx < a->nz && a->indices[aidx] < a0)
        aidx++;
    while (bidx < b->nz && b->indices[bidx] < b0)
        bidx++;

    int maxsz = a->nz - aidx + b->nz - bidx;

    // XXX beware of stack overflow
    TYPE tmp_values[maxsz];
    int tmp_indices[maxsz];
    int tmp_nz = 0;

    // original code
    int ai, bi;

    int *aidxs = a->indices;
    int *bidxs = b->indices;

    double *avals = a->values;
    double *bvals = b->values;

    ai = (aidx < a->nz) ? aidxs[aidx] : INT32_MAX;
    bi = (bidx < b->nz) ? bidxs[bidx] : INT32_MAX;

    while (aidx < a->nz || bidx < b->nz) {
        if (ai == bi) {
            // add and increment both.
            tmp_values[tmp_nz] = avals[aidx] + bvals[bidx] * bscale;
            tmp_indices[tmp_nz] = ai;
            tmp_nz++;
            aidx++;
            ai = (aidx < a->nz) ? aidxs[aidx] : INT32_MAX;
            bidx++;
            bi = (bidx < b->nz) ? bidxs[bidx] : INT32_MAX;
        } else if (ai < bi) {
            tmp_values[tmp_nz] = avals[aidx];
            tmp_indices[tmp_nz] = ai;
            tmp_nz++;
            aidx++;
            ai = (aidx < a->nz) ? aidxs[aidx] : INT32_MAX;
        } else {
            // bi < ai
            tmp_values[tmp_nz] = bvals[bidx] * bscale;
            tmp_indices[tmp_nz] = bi;
            tmp_nz++;
            bidx++;
            bi = (bidx < b->nz) ? bidxs[bidx] : INT32_MAX;
        }
    }

    svecd_ensure_capacity(x, tmp_nz);

    memcpy(x->indices, tmp_indices, sizeof(int)*tmp_nz);
    memcpy(x->values, tmp_values, sizeof(TYPE)*tmp_nz);
    x->nz = tmp_nz;
}

/////////////////////////////////////////////////
// matrix operations

smatd_t *smatd_create(int nrows, int ncols)
{
    smatd_t *m = calloc(1, sizeof(smatd_t));
    m->nrows = nrows;
    m->ncols = ncols;

    // don't allocate anything for the rows yet.
    m->rows = calloc(nrows, sizeof(svecd_t));
    for (int i = 0; i < nrows; i++) {
        m->rows[i].length = ncols;
    }

    return m;
}

smatd_t *smatd_create_data(int nrows, int ncols, const TYPE *data)
{
    smatd_t *m = smatd_create(nrows, ncols);
    int pos = 0;

    for (int row = 0; row < nrows; row++) {
        for (int col = 0; col < ncols; col++) {
            smatd_set(m, row, col, data[pos]);
            pos++;
        }
    }

    return m;
}

smatd_t *smatd_copy(smatd_t *a)
{
    smatd_t *x = smatd_create(a->nrows, a->ncols);

    for (int i = 0; i < a->nrows; i++) {
        svecd_ensure_capacity(&x->rows[i], a->rows[i].nz);

        x->rows[i].nz = a->rows[i].nz;
        memcpy(x->rows[i].indices, a->rows[i].indices, sizeof(int)*a->rows[i].nz);
        memcpy(x->rows[i].values, a->rows[i].values, sizeof(TYPE)*a->rows[i].nz);
    }

    return x;
}

smatd_t *smatd_identity(int nrows, int ncols)
{
    smatd_t *m = smatd_create(nrows, ncols);

    int mx = nrows < ncols ? nrows : ncols;

    for (int i = 0; i < mx; i++)
        smatd_set(m, i, i, 1);

    return m;
}

void smatd_destroy(smatd_t *m)
{
    if (m == NULL)
        return;

    for (int i = 0; i < m->nrows; i++) {
        svecd_t *row = &m->rows[i];
        if (row) {
            free(row->indices);
            free(row->values);
            // don't free row; it's allocated as an array
        }
    }

    free(m->rows);
    free(m);
}

void smatd_set(smatd_t *m, int row, int col, TYPE val)
{
    svecd_set(&m->rows[row], col, val);
}

TYPE smatd_get(smatd_t *m, int row, int col)
{
    return svecd_get(&m->rows[row], col);
}

void smatd_print(smatd_t *m, const char *fmt)
{
    for (int i = 0; i < m->nrows; i++) {
        for (int j = 0; j < m->ncols; j++) {
            printf(fmt, smatd_get(m, i, j));
        }
        printf("\n");
    }
}

void smatd_debug(smatd_t *m, const char *fmt)
{
    printf("Matrix %d x %d\n", m->nrows, m->ncols);

    for (int i = 0; i < m->nrows; i++) {
        svecd_t *row = &m->rows[i];
        assert(row->length == m->ncols);

        for (int pos = 0; pos < row->nz; pos++) {
            printf("(%d: %d ", pos, row->indices[pos]);
            printf(fmt, row->values[pos]);
            printf(") ");
        }
        printf("\n");
    }
}

// CAUTION: Returns the internal representation of this
// matrix. Modifications of this vector will affect the matrix. Do not
// attempt to destroy the underlying vector.
svecd_t *smatd_get_row_volatile(smatd_t *m, int row)
{
    return &m->rows[row];
}

// unlike get_row, this returns a copy of the underlying data. This
// vector must be destroyed by the caller.
svecd_t *smatd_copy_column(smatd_t *m, int column)
{
    svecd_t *v = svecd_create(m->nrows);

    for (int row = 0; row < m->nrows; row++) {
        svecd_set(v, row, svecd_get(&m->rows[row], column));
    }

    return v;
}

// multiply using accumulation. Advantage is that we can skip
// completely over many zero cells. Disadvantage is that we skip
// around rows of b quite a lot and must accumulate the result (which
// means we have to work at keeping CSR order.)
//
// ---a--- ---x---  = a0*x + a1*y + a2*z
// ---b--- ---y---  = b0*x + b1*y + b2*z
// ---c--- ---z---  = c0*x + c1*y + c2*z
//
// On highly sparse matrices, this is much faster than the dotproduct
// method. On dense matrices, it's not a lot slower ==> make this the
// default.
smatd_t *smatd_multiply(smatd_t *a, smatd_t *b)
{
    assert(a->ncols == b->nrows);

    int xrows = a->nrows;
    int xcols = b->ncols;

    smatd_t *x = smatd_create(xrows, xcols);

    for (int i = 0; i < a->nrows; i++) {
        svecd_t *arow = &a->rows[i];
        svecd_t *xrow = &x->rows[i];

        // QQQ: Could also accumulate xrow using a dense vector, then collapse at the end.

        for (int pos = 0; pos < arow->nz; pos++) {
            TYPE v = arow->values[pos];
            int acol = arow->indices[pos];
            if (v==0)
                continue;

            assert(acol < b->nrows);
            svecd_t *brow = &b->rows[acol];
            svecd_add_i0(xrow, xrow, brow, v, 0, 0);
        }
    }

    return x;
}

// multiply using dot products.
smatd_t *smatd_multiply_dpmethod(smatd_t *a, smatd_t *b)
{
    assert(a->ncols == b->nrows);

    int xrows = a->nrows;
    int xcols = b->ncols;

    smatd_t *x = smatd_create(xrows, xcols);
    for (int col = 0; col < xcols; col++) {
        svecd_t *bcol = smatd_copy_column(b, col);

        for (int row = 0; row < xrows; row++) {
            svecd_t *arow = &a->rows[row];

            smatd_set(x, row, col, svecd_dot_product(arow, bcol));
        }

        svecd_destroy(bcol);
    }

    return x;
}

smatd_t *smatd_transpose(smatd_t *a)
{
    smatd_t *x = smatd_create(a->ncols, a->nrows);
    int cursors[x->nrows];
    memset(cursors, 0, sizeof(cursors));

    for (int inrow = 0; inrow < a->nrows; inrow++) {
        svecd_t *arow = &a->rows[inrow];

        for (int pos = 0; pos < arow->nz; pos++) {
            int incol = arow->indices[pos];

            svecd_t *xrow = &x->rows[incol];
            svecd_ensure_capacity(xrow, cursors[incol]+1);
            xrow->indices[xrow->nz] = inrow;
            xrow->values[xrow->nz] = arow->values[pos];
            xrow->nz++;
        }
    }

    return x;
}

smatd_t *smatd_upper_right(smatd_t *a)
{
    smatd_t *x = smatd_create(a->nrows, a->ncols);
    for (int row = 0; row < a->nrows; row++) {
        svecd_t *av = &a->rows[row];
        svecd_t *xv = &x->rows[row];

        for (int aidx = 0; aidx < av->nz; aidx++) {
            if (av->indices[aidx] >= row) {
                xv->length = a->ncols;
                svecd_ensure_capacity(xv, av->nz - aidx);
                xv->nz = av->nz - aidx;
                memcpy(xv->indices, &av->indices[aidx], xv->nz * sizeof(int));
                memcpy(xv->values, &av->values[aidx], xv->nz * sizeof(TYPE));
                break;
            }
        }
    }
    return x;
}

// explanation of Cholesky approach:
//
// Consider matrix M (which we wish to factor).
//
// M =  [ A  B ]   Where A is 1x1, B is 1xN, C is NxN.
//      [ B' C ]
//
// The Cholesky factorization for this must be of the form:
//
// [ X  0 ] [ X  Y ] (where X is 1x1, Y is 1xN, etc.)
// [ Y' Z'] [ 0  Z ]
//
//  We must have X*X = A, or X = sqrt(A). Similarly, we must have Y =
//  B/sqrt(A). These choices suffice to completely (and correctly)
//  result in upper left portion (A, B, B') of M.
//
// But these choices for X and Y also contribute terms to the C parts
// of matrix M (due to the product of Y' and Y). So we subtract these
// contributions from C and then recursively factor what's left over.
//
// Specifically, C = Y'Y + Z'Z, so Z'Z = C - Y'Y. (We will then
// recursively factor Z'Z.)
smatd_chol_t *smatd_chol(smatd_t *a)
{
    // create a new matrix that is the upper-right of A.
    smatd_t *u = smatd_upper_right(a);

    int is_spd = (a->nrows == a->ncols);

    // recursively factor each "outer" upper-left shell (X,Y,Z),
    // modifying the internal bits.
    for (int i = 0; i < a->nrows; i++) {
        svecd_t *urowi = &u->rows[i];

        double d = svecd_get(urowi, i);
        is_spd &= (d > 0);
        d = sqrt(d);

        svecd_scale(urowi, 1.0 / d);

        // iterate over the non-zero elements... each non-zero element
        // will contribute something to block C, which we need to subtract.
        for (int pos = 0; pos < urowi->nz; pos++) {
            int j = urowi->indices[pos];
            if (j > i) {
                // apply an update to a row below us
                double s = urowi->values[pos];

                svecd_t *urowj = &u->rows[j];
//                assert(urowj->indices[0] == j);
//                assert(urowi->indices[pos] == j);
                svecd_add_i0_x(urowj, urowj, urowi, -s, 0, pos);
            }
        }
    }

    smatd_chol_t *chol = calloc(1, sizeof(smatd_chol_t));
    chol->is_spd = is_spd;
    chol->u = u;
    return chol;
}

void smatd_chol_destroy(smatd_chol_t *chol)
{
    smatd_destroy(chol->u);
    free(chol);
}

void smatd_ltriangle_solve(smatd_t *L, const TYPE *b, TYPE *x)
{
    for (int i = 0; i < L->ncols; i++) {
        double bi = b[i];

        svecd_t *Lrowi = &L->rows[i];

        // this row should end with a value along the diagonal.
        // NB: rank-deficient matrices could have a zero here?
        assert(Lrowi->nz > 0);
        assert(Lrowi->indices[Lrowi->nz-1] == i);

        TYPE diag = Lrowi->values[Lrowi->nz-1];

        // compute dot product of Lrowi and the first (i-1) elements
        // of y.  Note: exploit fact that l is exactly
        // triangular. Don't need range checks.
        for (int pos = 0; pos < Lrowi->nz - 1; pos++)
            bi -= Lrowi->values[pos]*x[Lrowi->indices[pos]];

        x[i] = bi / diag;
    }
}

// Solve the triangular system Lx = b, where L is provided in the form
// of its transpose (U). The math for this follows from solving the
// system: x'U = b' and proceeds recursively (similar to the cholesky
// factorization).
void smatd_ltransposetriangle_solve(smatd_t *u, const TYPE *b, TYPE *x)
{
    int n = u->ncols;

    memcpy(x, b, n*sizeof(TYPE));

    for (int i = 0; i < n; i++) {
        svecd_t *urowi = &u->rows[i];
        x[i] = x[i] / urowi->values[0];

        for (int pos = 1; pos < urowi->nz; pos++) {
            int uidx = urowi->indices[pos];
            x[uidx] -= x[i] * urowi->values[pos];
        }
    }
}

void smatd_utriangle_solve(smatd_t *u, const TYPE *b, TYPE *x)
{
    for (int i = u->ncols-1; i >= 0; i--) {
        double bi = b[i];

        svecd_t *urowi = &u->rows[i];

        // this row should end with a value along the diagonal.
        // NB: rank-deficient matrices could have a zero here?
        assert(urowi->nz > 0);
        assert(urowi->indices[0] == i);

        TYPE diag = urowi->values[0];

        // compute dot product of urowi and the last (i-1) elements of
        // y.  Note: exploit fact that u is exactly triangular-- the
        // first element will be the diagonal (which we want to skip)
        for (int pos = 1; pos < urowi->nz ; pos++)
            bi -= urowi->values[pos]*x[urowi->indices[pos]];

        x[i] = bi / diag;
    }
}

// Solve the system Ax=b. User provides storage for x.
void smatd_chol_solve(smatd_chol_t *chol, const TYPE *b, TYPE *x)
{
    // U'Ux = b
    // U'y = b

    smatd_t *u = chol->u;

    TYPE y[u->ncols];

        // back solve U'y = b
        // smatd_t *l = smatd_transpose(u);
        // smatd_ltriangle_solve(l, b, y);
        // smatd_destroy(l);

    // this replacement is around 10x faster than transposing and using ltriangle_solve
    smatd_ltransposetriangle_solve(u, b, y);

    // back solve Ux = y
    smatd_utriangle_solve(u, y, x);
}

// basically the same idea as chol, except we don't assume symmetry.
//
// Begin with a block decomposition of M, with A = 1x1, B 1xN, C Nx1, D NxN.
// Consider just the LDU decomposition of this block decomposition:
//
// M = [ A B ] = [ I  0  ] [ X  0 ] [ I  Z ] = [ X       XZ   ]
//     [ C D ]   [ W  I  ] [ 0  Y ] [ 0  I ]   [ WX   WXZ + Y ]
//
// X = A
// Z = inv(A)*B
// W = C*inv(A)
// Y = D - C*inv(A)*B
//
// Or, in other words, we want to transform like this:
//
// [ A B ] => [ X Z ] = [ A            inv(A)*B     ]
// [ C D ]    [ W Y ]   [ C*inv(A)   D - C*inv(A)*B ]
//
// We then recurse on the Y submatrix.
//
// The result will be a single matrix with the L, D, and U all packed
// together (I and 0 terms omitted.)
smatd_ldu_t *smatd_ldu(smatd_t *a)
{
    smatd_t *ldu = smatd_copy(a);

    // recursively do LDU decomposition for submatrix of M with upper
    // corner at i,i:
    for (int i = 0; i < ldu->nrows; i++) {
        svecd_t *ldurowi = &ldu->rows[i];

        // get A block
        TYPE d = svecd_get(ldurowi, i);
        TYPE invd = 1.0 / d;

        // handle Z sub-block: the rest of the row is scaled by 1/d
        svecd_scale_i0(ldurowi, invd, i+1);

        for (int j = i+1; j < ldu->nrows; j++) {
            // handle W sub-block
            // INEFFICIENT?
            TYPE v = svecd_get(&ldu->rows[j], i);
            svecd_set(&ldu->rows[j], i, v * invd);

            // handle Y sub-block--- subtract C*inv(A)*B
            if (v == 0)
                continue;

            // Note that we've already scaled this ldurowi by invd.
            svecd_add_i0(&ldu->rows[j], &ldu->rows[j], ldurowi, -v, 0, i+1);
        }
    }

    smatd_ldu_t *sldu = calloc(1, sizeof(smatd_ldu_t));
    sldu->ldu = ldu;

    return sldu;
}

void smatd_ldu_destroy(smatd_ldu_t *sldu)
{
    if (sldu==NULL)
        return;
    smatd_destroy(sldu->ldu);
    free(sldu);
}

void smatd_ldu_get(smatd_ldu_t *sldu, smatd_t **l_out, smatd_t **d_out, smatd_t **u_out)
{
    smatd_t *ldu = sldu->ldu;

    int R = ldu->nrows < ldu->ncols ? ldu->nrows : ldu->ncols;

    smatd_t *l = smatd_create(ldu->nrows, R);
    smatd_t *d = smatd_create(R, R);
    smatd_t *u = smatd_create(R, ldu->ncols);

    for (int row = 0; row < ldu->nrows; row++) {
        svecd_t *ldurow = &ldu->rows[row];

        int lnz = 0;
        int unz = 0;
        int upos = 0;

        for (int pos = 0; pos < ldurow->nz; pos++) {

            if (ldurow->indices[pos] == row) {
                // diagonal element? add to D.
                if (row < R) {
                    svecd_t *drow = &d->rows[row];

                    svecd_ensure_capacity(drow, 1);
                    drow->nz = 1;
                    drow->indices[0] = row;
                    drow->values[0] = ldurow->values[pos];
                }
                continue;
            } else if (ldurow->indices[pos] < row) {
                lnz++;
                continue;
            } else {
                // index must be into the U matrix (indices[pos] > row)
                upos = pos;
                unz = ldurow->nz - upos;
                break; // don't need to do any more searching.
            }
        }

        if (row < l->nrows) {
            svecd_t *lrow = &l->rows[row];
            svecd_ensure_capacity(lrow, lnz+1);
            memcpy(&lrow->indices[0], &ldurow->indices[0], lnz*sizeof(int));
            memcpy(&lrow->values[0], &ldurow->values[0], lnz*sizeof(TYPE));
            if (row < lrow->length) {
                lrow->indices[lnz] = row;
                lrow->values[lnz] = 1;
                lrow->nz = lnz+1;
            }
        }

        if (row < u->nrows) {
            svecd_t *urow = &u->rows[row];
            svecd_ensure_capacity(urow, unz+1);
            memcpy(&urow->indices[1], &ldurow->indices[upos], unz*sizeof(int));
            memcpy(&urow->values[1], &ldurow->values[upos], unz*sizeof(TYPE));
            if (row < urow->length) {
                urow->indices[0] = row;
                urow->values[0] = 1;
                urow->nz = unz+1;
            }
        }
    }

    *l_out = l;
    *d_out = d;
    *u_out = u;
}

int smatd_nz(smatd_t *A)
{
    int nz = 0;

    for (int rowi = 0; rowi < A->nrows; rowi++) {
        svecd_t *row = &A->rows[rowi];

        nz += row->nz;
    }

    return nz;
}
