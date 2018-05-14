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
#include <stdint.h>
#include <string.h>
#include <assert.h>

#include "rs.h"

// brute-force version used to build a lookup table.
static uint32_t rs_mul_internal(uint32_t nbits, uint32_t p, uint32_t a, uint32_t b)
{
    uint32_t acc = 0;

    // long multiplication by bits of a.
    for (int i = nbits-1; i >= 0; i--) {
        if (a & (1 << i))
            acc ^= (b << i);
    }

    // long division by p
    for (int i = 2*nbits; i >= nbits; i--) {
        if (acc & (1 << i)) {
            acc ^= (p << (i - nbits));
        }
    }

    return acc;
}

/*
static void print_mat(uint16_t *M, int nrows, int ncols)
{
    for (int i = 0; i < nrows; i++) {
        for (int j = 0; j < ncols; j++) {
            printf("%3d ", M[i*ncols + j]);
        }
        printf("\n");
    }
}
*/

static inline uint32_t rs_mul(rs_t *rs, uint32_t a, uint32_t b)
{
    return rs->M[a*rs->N + b];
}

static inline uint32_t rs_pow(rs_t *rs, uint32_t a, uint32_t b)
{
    uint32_t v = 1;

    uint32_t mask = 1;
    uint32_t powb = a;

    while (b) {
        if (b & mask)  {
            v = rs_mul(rs, v, powb);
            b ^= mask;
        }

        powb = rs_mul(rs, powb, powb);
        mask <<= 1;
    }

//    for (int i = 0; i < b; i++)
//        v = rs_mul(rs, v, a);

    return v;
}

// if poly==0, a default poly is used.
rs_t *rs_create(int nbits, int poly, int datasz, int encsz)
{
    assert(nbits > 0 && nbits < 16);
    assert(encsz >= datasz); // though encsz==datasz is pretty silly.
    assert(encsz <= (1 << nbits));
    assert(datasz > 0);

    if (poly == 0) {
        int polys[] = { 0x0000,
                        0x0002, 0x0007, 0x000b, 0x0013,
                        0x0025, 0x0043, 0x0083, 0x011b,
                        0x0203, 0x0409, 0x0805, 0x1009,
                        0x201b, 0x4021 };
        assert(nbits <= sizeof(polys) / sizeof(int));

        poly = polys[nbits];
    }

    rs_t *rs = calloc(1, sizeof(rs_t));
    rs->nbits = nbits;
    rs->poly = poly;

    int N = 1 << rs->nbits;
    rs->N = N;

    // create multiplication look-up table
    rs->M = malloc(N*N*2);
    rs->inv = malloc(N*2);

    for (int a = 0; a < N; a++) {
        for (int b = a; b < N; b++) {
            rs->M[a*N+b] = rs_mul_internal(rs->nbits, rs->poly, a, b);
            rs->M[b*N+a] = rs->M[a*N+b];
            if (rs->M[a*N+b] == 1) {
                rs->inv[a] = b;
                rs->inv[b] = a;
            }
        }
    }

//    print_mat(rs->M, N, N);

    // create encoding table
    rs->datasz = datasz;
    rs->encsz = encsz;
    rs->E = malloc(rs->datasz*rs->encsz*2);
    for (int row = 0; row < rs->encsz; row++) {
        for (int col = 0; col < rs->datasz; col++) {
            // vandermonde matrix.
            rs->E[rs->datasz*row + col] = rs_pow(rs, row, col);
        }
    }

//    print_mat(rs->E, rs->encsz, rs->datasz);

    return rs;
}

// validate the polynomial
int rs_validate(rs_t *rs)
{
    int N = 1 << rs->nbits;

    // verify that the poly is irreducible by making sure each row of
    // M contains every element.
    for (int a = 1; a < N; a++) {
        uint8_t present[N];
        memset(present, 0, N);

        for (int b = 0; b < N; b++) {
            if (present[rs->M[a*N+b]]) {
                return 0;
            }
            present[rs->M[a*N+b]] = 1;
        }
    }

    // validate inverses
    for (int a = 1; a < N; a++) {
        int b = rs->inv[a];

        if (rs->M[a*N+b] != 1)
            return 0;
    }

    return 1;
}

void rs_encode(rs_t *rs, const uint16_t *data, uint16_t *enc)
{
    memset(enc, 0, 2*rs->encsz);

    for (int i = 0; i < rs->encsz; i++) {
        int acc = 0;
        for (int j = 0; j < rs->datasz; j++) {
            acc ^= rs_mul(rs, rs->E[i*rs->datasz + j], data[j]);
        }
        enc[i] = acc;
    }
}

void rs_destroy(rs_t *rs)
{
    if (!rs)
        return;

    free(rs->M);
    free(rs->E);
    free(rs);
}

rs_erasure_decoder_t *rs_erasure_decoder_create(rs_t *rs, const uint16_t *encidx)
{
    rs_erasure_decoder_t *rsd = calloc(1, sizeof(rs_erasure_decoder_t));
    rsd->rs = rs;

    // construct the encoding matrix that was used by selecting the
    // rows of E given by encidx
    uint16_t *E = calloc(2, rs->datasz * rs->datasz);
    for (int i = 0; i < rs->datasz; i++) {
        for (int j = 0; j < rs->datasz; j++) {
            E[i*rs->datasz + j] = rs->E[encidx[i]*rs->datasz + j];
        }
    }

    // we will now compute the inverse of D using Gaussian
    // elimination. Initialize DI to be the identity matrix.
    uint16_t *D = calloc(2, rs->datasz * rs->datasz);
    rsd->D = D;
    for (int i = 0; i < rs->datasz; i++) {
        D[i*rs->datasz + i] = 1;
    }

    for (int rowi = 0; rowi < rs->datasz; rowi++) {

        // rank deficient?
        if (E[rowi*rs->datasz+rowi] == 0) {
            printf("TROUBLE:\n");
            exit(-1);
        }

        // normalize this row (make diagonal value a '1')
        int coeff = rs->inv[E[rowi*rs->datasz+rowi]];
        for (int col = rowi; col < rs->datasz; col++)
            E[rowi*rs->datasz+col] = rs_mul(rs, coeff, E[rowi*rs->datasz+col]);
        for (int col = 0; col < rs->datasz; col++)
            D[rowi*rs->datasz+col] = rs_mul(rs, coeff, D[rowi*rs->datasz+col]);

        for (int rowj = rowi + 1; rowj < rs->datasz; rowj++) {
            // eliminate column 'rowi' from rowj

            int coeff = rs->inv[E[rowi*rs->datasz+rowi]] * E[rowj*rs->datasz+rowi];

            // add -rowi*coeff to rowj.
            for (int col = rowi; col < rs->datasz; col++)
                E[rowj*rs->datasz+col] ^= rs_mul(rs, coeff, E[rowi*rs->datasz+col]);
            for (int col = 0; col < rs->datasz; col++)
                D[rowj*rs->datasz+col] ^= rs_mul(rs, coeff, D[rowi*rs->datasz+col]);
        }
    }

    for (int rowi = rs->datasz-1; rowi >= 0; rowi--) {
        for (int rowj = 0; rowj < rowi; rowj++) {
            // eliminate column rowi from rowj.
            int coeff = E[rowj*rs->datasz+rowi];
            for (int col = rowi; col < rs->datasz; col++)
                E[rowj*rs->datasz+col] ^= rs_mul(rs, coeff, E[rowi*rs->datasz+col]);
            for (int col = 0; col < rs->datasz; col++)
                D[rowj*rs->datasz+col] ^= rs_mul(rs, coeff, D[rowi*rs->datasz+col]);
        }
    }

    return rsd;
}

void rs_erasure_decoder_decode(rs_erasure_decoder_t *rsd, const uint16_t *enc, uint16_t *data)
{
    rs_t *rs = rsd->rs;
    int N = rs->datasz;

    // now, actually do the decode.
    for (int i = 0; i < N; i++) {
        int acc = 0;
        for (int j = 0; j < N; j++) {
            acc ^= rs_mul(rs, rsd->D[i*N+j], enc[j]);
        }
        data[i] = acc;
    }
}

void rs_erasure_decoder_destroy(rs_erasure_decoder_t *rsd)
{
    if (!rsd)
        return;
    free(rsd->D);
    free(rsd);
}

// returns number of non-zero terms in the remainder Q(x)/E(x). Zero
// indicates successful decoding.
int rs_bw_decoder(rs_t *rs, const uint16_t *encidx, int sz, const uint16_t *enc, uint16_t *data)
{
    assert(sz >= rs->datasz);
    uint16_t *A = calloc(2, sz*sz);

    int N = rs->datasz;
    int Z = (sz - N) / 2;

    int Qsz = Z+N; // how many Q terms?
    int Esz = Z;   // how many E terms?

    if (sz == Qsz + Esz + 1) {
        // we could have one "extra" row. Should we bump up Qsz or shrink sz?
        Qsz++; // sz--;
    }
    assert(sz == (Qsz + Esz)); // could be 1 greater, then drop a row?

    //
    // Q(x) = P(x)E(x)
    //
    // where P is a polynomial of order rs->datasz whose coefficients
    //   are the data we encoded, and whose values at P(0),P(1),P(2)
    //   we transmitted and received (possibly corrupted).
    //
    // E(x) is a polynomial whose roots give the indices of the
    // corrupted samples. (It can be written (x-r0)(x-r1)...(x-rZ),
    // note that the highest-order term has coefficient 1.)
    //
    //   E(x) = x^0*e0 + x^1*e1 + x^2*e2 + ... + x^Z   (Z=sz - N)
    //
    // Q will be of order N+Z.
    //
    //   Q(x) = x^0*q0 + x^1*q1 + x^2*q2 + ... + x^(N+Z)*q{N+Z}
    //
    // Q(x) = P(x)E(x)
    // x^0*q0 + x^1*q1 + x^2*q2 + ... + x^(N+Z)*q{N+Z}  =  P(x) (x^0*e0 + x^1*e1 + x^2*e2 + ... + x^Z)
    //
    // moving unknowns (q's, e's) to LHS
    //
    // x^0*q0 + x^1*q1 + x^2*q2 + ... + x^(N+Z)*q{N+Z} - P(x)x^0*e0 - P(x)x^1*e1 - ... = P(x)x^Z
    //
    // [x^0 x^1 x^2 ... x^(N+Z) -P(x)x^0 -P(x)x^1 ... ] [q0 q1 ... e0 e1 ...]' = P(x)x^Z

    uint16_t R[sz]; // the right hand side

    for (int i = 0; i < sz; i++) {
        for (int j = 0; j < Qsz; j++)
            // vandermonde. coefficients of qi
            A[i*sz+j] = rs_pow(rs, encidx[i], j);
        for (int j = 0; j < Esz; j++) {
            A[i*sz+Qsz+j] = rs_mul(rs, rs_pow(rs, encidx[i], j), enc[i]);
        }

        R[i] = rs_mul(rs, rs_pow(rs, encidx[i], Z), enc[i]);
    }

    // now solve: A [ q0 q1 ... qN  e0 e1 ... eZ ]' = R

    for (int rowi = 0; rowi < sz; rowi++) {

        // rank deficient?
        //
        // If we find a row that looks like it needs a pivot, we just
        // process it anyway... zeros just result in NOPs.
        //
/*
        if (0 && A[rowi*sz + rowi] == 0) {
            // XXX maybe all this is pointless, since our LUT will say 0^-1=0.

            if (rowi >= rs->datasz) {
                // pivot
                int piv = -1;
                for (int k = rowi+1; k < sz; k++) {
                    if (A[k*sz + rowi] != 0) {
                        piv = k;
                        break;
                    }
                }

//                print_mat(A, sz, sz);
//                printf("\n");

                if (piv < 0) {
                    // no pivot found. This may mean that the entire
                    // rest of the matrix is zero, which means FEWER
                    // errors were found than could be corrected.
                    for (int i = rowi; i < sz; i++) {
                        for (int j = rowi; j < sz; j++) {
                            if (A[i*sz+j] != 0) {
//                                printf("No pivot found for non-zero sub-block. Fatal.\n");
                                continue;
//                                free(A);
//                                return sz;
                            }
                        }
                    }

                    // if we got here, all is okay: the whole
                    // remainder of the matrix is zero.
                } else {
//                    printf("piv %d\n", piv);
                    // found a pivot.
                    uint16_t Atmp[sz];
                    memcpy(Atmp, &A[rowi*sz], 2*sz);
                    memcpy(&A[rowi*sz], &A[piv*sz], 2*sz);
                    memcpy(&A[piv*sz], Atmp, 2*sz);

                    uint16_t Rtmp = R[rowi];
                    R[rowi] = R[piv];
                    R[piv] = Rtmp;
                }

            } else {
                printf("TROUBLE: rank deficient too early!\n");
                exit(-1);
            }
        }
*/

        // normalize this row (make diagonal value a '1')
        int coeff = rs->inv[A[rowi*sz+rowi]];
        for (int col = rowi; col < sz; col++)
            A[rowi*sz+col] = rs_mul(rs, coeff, A[rowi*sz+col]);
        R[rowi] = rs_mul(rs, coeff, R[rowi]);

        for (int rowj = rowi + 1; rowj < sz; rowj++) {
            // eliminate column 'rowi' from rowj
            int coeff = rs->inv[A[rowi*sz+rowi]] * A[rowj*sz+rowi];

            // add -rowi*coeff to rowj.
            for (int col = rowi; col < sz; col++)
                A[rowj*sz+col] ^= rs_mul(rs, coeff, A[rowi*sz+col]);
            R[rowj] ^= rs_mul(rs, coeff, R[rowi]);
        }
    }

    for (int rowi = sz-1; rowi >= 0; rowi--) {
        for (int rowj = 0; rowj < rowi; rowj++) {
            // eliminate column rowi from rowj.
            int coeff = A[rowj*sz+rowi];
            for (int col = rowi; col < sz; col++)
                A[rowj*sz+col] ^= rs_mul(rs, coeff, A[rowi*sz+col]);
            R[rowj] ^= rs_mul(rs, coeff, R[rowi]);
        }
    }

    // perform long division of Q(x) / E(x), this time keeping the
    // *quotient*. (The remainder should be exactly zero; what does it
    // mean if we get non-zero?)
    //
    // Remember that E(x) has one implicit term: 1*x^Z.
    //
    // process terms of Q from largest-to-smallest order
    uint16_t *Q = R;
    uint16_t *E = &R[Qsz];

    for (int i = Qsz-1; i >= Esz; i--) {
        int coeff = Q[i]; // get Q_i
        data[i - Esz] = coeff;

        // subtract coeff*E(x)*x^(i-Esz) from Q(x)
        for (int j = 0; j < Esz; j++) {
            Q[j+i-Esz] ^= rs_mul(rs, coeff, E[j]);
        }
        Q[i] = 0; // this term is canceled, by construction

    }

    free(A);

    int nerr = 0;
    for (int i = 0; i < Qsz; i++)
        if (Q[i])
            nerr++;
    return nerr;
}
