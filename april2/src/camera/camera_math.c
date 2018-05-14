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

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>

#include "common/matd.h"
#include "common/matd_coords.h"
#include "common/zarray.h"
#include "common/math_util.h"
#include "common/smatd.h"
#include "camera_math.h"

#define GET_MAX(a,b) (a<=b ? b : a)
#define GET_MIN(a,b) (a<=b ? a : b)
#define TRUE 1
#define FALSE 0

#define EL(m, row,col) (m)->data[((row)*(m)->ncols + (col))]


matd_t * camera_math_triangulate(matd_t * Pleft, matd_t *xy_left,
                                 matd_t * Pright, matd_t *xy_right)
{
    matd_t *A = matd_create(4,4);

    for (int i = 0; i < 4; i++) {
        EL(A,0,i) = EL(Pleft ,2,i)*xy_left->data[0]  - EL(Pleft ,0,i);
        EL(A,1,i) = EL(Pleft ,2,i)*xy_left->data[1]  - EL(Pleft ,1,i);
        EL(A,2,i) = EL(Pright,2,i)*xy_right->data[0] - EL(Pright,0,i);
        EL(A,3,i) = EL(Pright,2,i)*xy_right->data[1] - EL(Pright,1,i);
    }

    matd_svd_t svd = matd_svd(A);

    matd_t *xyz = matd_create(3, 1);
    for (int i = 0; i < 3; i++) {
        EL(xyz,i,0) = EL(svd.V,i,3) / EL(svd.V,3,3);
    }

    matd_destroy(svd.U);
    matd_destroy(svd.S);
    matd_destroy(svd.V);
    matd_destroy(A);

    // points behind the plane are bad matches
/*
  if (xyz->data[2] < 0) {
  matd_destroy(xyz);
  return NULL;
  }
*/
    return xyz;
}

matd_t * camera_math_triangulate_iter(matd_t * Pleft, matd_t *xy_left,
                                 matd_t * Pright, matd_t *xy_right)
{
    double wi = 1, wi1 = 1;
    matd_t *X = matd_create(4,1);
    matd_t *X_ = camera_math_triangulate(Pleft,xy_left,Pright,xy_right);
    X->data[0] = X_->data[0];
    X->data[1] = X_->data[1];
    X->data[2] = X_->data[2];
    X->data[3] = 1;
    for(int i=0; i<10; i++){
        double p2x = Pleft->data[4]*X->data[0] + Pleft->data[4]*X->data[1] + Pleft->data[4]*X->data[2] + Pleft->data[4]*X->data[3];
        double p2x1 = Pright->data[4]*X->data[0] + Pright->data[4]*X->data[1] + Pright->data[4]*X->data[2] + Pright->data[4]*X->data[3];

        //breaking points
        if(fabsf(wi - p2x) <= 1e-9 && fabsf(wi1 - p2x1) <= 1e-9){
            break;
        }
        printf("%d\n",i);
        wi = p2x;
        wi1 = p2x1;

        matd_t *A = matd_create(4,3);
        for(int i=0; i<3; i++){
            EL(A,0,i) = (EL(Pleft,2,i)*xy_left->data[0]  - EL(Pleft ,0,i))/wi;
            EL(A,1,i) = (EL(Pleft,2,i)*xy_left->data[1]  - EL(Pleft ,1,i))/wi;
            EL(A,2,i) = (EL(Pright,2,i)*xy_right->data[0]  - EL(Pright ,0,i))/wi1;
            EL(A,3,i) = (EL(Pright,2,i)*xy_right->data[1]  - EL(Pright ,1,i))/wi1;
        }
        matd_t *b = matd_create(4,1);
        EL(b,0,0) = (xy_left->data[0] * EL(Pleft,2,3) - EL(Pleft,0,3))/wi;
        EL(b,1,0) = (xy_left->data[1] * EL(Pleft,2,3) - EL(Pleft,1,3))/wi;
        EL(b,2,0) = (xy_right->data[0] * EL(Pright,2,3) - EL(Pright,0,3))/wi1;
        EL(b,3,0) = (xy_right->data[0] * EL(Pright,2,3) - EL(Pright,1,3))/wi1;

        matd_svd_t svd = matd_svd(A);
        matd_t *S_inv = matd_create(3,3);
        EL(S_inv,0,0) = 1/EL(svd.S,0,0);
        EL(S_inv,1,1) = 1/EL(svd.S,1,1);
        EL(S_inv,2,2) = 1/EL(svd.S,2,2);

        matd_t *U = matd_select(svd.U,0,3,0,2);
        matd_t *X_ = matd_op("M*M*M'*M",svd.V,S_inv,U,b);
        X->data[0] = X_->data[0];
        X->data[1] = X_->data[1];
        X->data[2] = X_->data[2];
        X->data[3] = 1;

        matd_destroy(U);
        matd_destroy(S_inv);
        matd_destroy(svd.U);
        matd_destroy(svd.S);
        matd_destroy(svd.V);
        matd_destroy(A);
        matd_destroy(b);
        matd_destroy(X_);
    }
    matd_destroy(X_);

    return X;
}


static double c_hypot(double a, double b)
{
    double r;
    if (fabs(a) > fabs(b)) {
        r = b/a;
        r = fabs(a)*sqrt(1+r*r);
    } else if (b != 0) {
        r = a/b;
        r = fabs(b)*sqrt(1+r*r);
    } else {
        r = 0.0;
    }
    return r;
}

static matd_t* SVD_get_V(matd_t* input)
{

    int m=input->nrows; //rows
    int n=input->ncols; //cols
    double A[m][n];
    int nu = GET_MIN(m,n);
    double s[GET_MIN(m+1,n)];
    memset(s,0,sizeof(double)*n*n);
    double U[m][nu];
    memset(U,0,sizeof(double)*m*nu);
    double V[n][n];
    memset(V,0,sizeof(double)*n*n);
    double e[n];
    memset(e,0,sizeof(double)*n);
    double work[m];
    memset(work,0,sizeof(double)*m);
    int wantu = TRUE;
    int wantv = TRUE;

    int nct = GET_MIN(m-1,n);
    int nrt = GET_MAX(0,GET_MIN(n-2,m));

    for(int i=0; i<m; i++){
        for(int j=0; j<n; j++){
            A[i][j] = matd_get(input,i,j);
        }
    }

    for(int k=0; k < GET_MAX(nct,nrt); k++){
        if (k < nct) {
            // Compute the transformation for the k-th column and
            // place the k-th diagonal in s[k].
            // Compute 2-norm of k-th column without under/overflow.
            s[k] = 0;
            for (int i = k; i < m; i++) {
                s[k] = c_hypot(s[k],A[i][k]);
            }
            if (s[k] != 0.0) {
                if (A[k][k] < 0.0) {
                    s[k] = -s[k];
                }
                for (int i = k; i < m; i++) {
                    A[i][k] /= s[k];
                }
                A[k][k] += 1.0;
            }
            s[k] = -s[k];
        }
        for (int j = k+1; j < n; j++) {
            if ((k < nct) & (s[k] != 0.0))  {
                // Apply the transformation.
                double t = 0;
                for (int i = k; i < m; i++) {
                    t += A[i][k]*A[i][j];
                }
                t = -t/A[k][k];
                for (int i = k; i < m; i++) {
                    A[i][j] += t*A[i][k];
                }
            }
            // Place the k-th row of A into e for the
            // subsequent calculation of the row transformation.
            e[j] = A[k][j];
        }
        if (wantu & (k < nct)) {
            // Place the transformation in U for subsequent back
            // multiplication.
            for (int i = k; i < m; i++) {
                U[i][k] = A[i][k];
            }
        }
        if (k < nrt) {

            // Compute the k-th row transformation and place the
            // k-th super-diagonal in e[k].
            // Compute 2-norm without under/overflow.
            e[k] = 0;
            for (int i = k+1; i < n; i++) {
                e[k] = c_hypot(e[k],e[i]);
            }
            if (e[k] != 0.0) {
                if (e[k+1] < 0.0) {
                    e[k] = -e[k];
                }
                for (int i = k+1; i < n; i++) {
                    e[i] /= e[k];
                }
                e[k+1] += 1.0;
            }
            e[k] = -e[k];
            if ((k+1 < m) & (e[k] != 0.0)) {

                // Apply the transformation.

                for (int i = k+1; i < m; i++) {
                    work[i] = 0.0;
                }
                for (int j = k+1; j < n; j++) {
                    for (int i = k+1; i < m; i++) {
                        work[i] += e[j]*A[i][j];
                    }
                }
                for (int j = k+1; j < n; j++) {
                    double t = -e[j]/e[k+1];
                    for (int i = k+1; i < m; i++) {
                        A[i][j] += t*work[i];
                    }
                }
            }
            if (wantv) {

                // Place the transformation in V for subsequent
                // back multiplication.
                for (int i = k+1; i < n; i++) {
                    V[i][k] = e[i];
                }
            }
        }
    }

    // Set up the final bidiagonal matrix or order p.

    int p = GET_MIN(n,m+1);
    if (nct < n) {
        s[nct] = A[nct][nct];
    }
    if (m < p) {
        s[p-1] = 0.0;
    }
    if (nrt+1 < p) {
        e[nrt] = A[nrt][p-1];
    }
    e[p-1] = 0.0;

    // If required, generate U.

    if (wantu) {
        for (int j = nct; j < nu; j++) {
            for (int i = 0; i < m; i++) {
                U[i][j] = 0.0;
            }
            U[j][j] = 1.0;
        }
        for (int k = nct-1; k >= 0; k--) {
            if (s[k] != 0.0) {
                for (int j = k+1; j < nu; j++) {
                    double t = 0;
                    for (int i = k; i < m; i++) {
                        t += U[i][k]*U[i][j];
                    }
                    t = -t/U[k][k];
                    for (int i = k; i < m; i++) {
                        U[i][j] += t*U[i][k];
                    }
                }
                for (int i = k; i < m; i++ ) {
                    U[i][k] = -U[i][k];
                }
                U[k][k] = 1.0 + U[k][k];
                for (int i = 0; i < k-1; i++) {
                    U[i][k] = 0.0;
                }
            } else {
                for (int i = 0; i < m; i++) {
                    U[i][k] = 0.0;
                }
                U[k][k] = 1.0;
            }
        }
    }

    if (wantv) {
        for (int k = n-1; k >= 0; k--) {
            if ((k < nrt) & (e[k] != 0.0)) {
                for (int j = k+1; j < nu; j++) {
                    double t = 0;
                    for (int i = k+1; i < n; i++) {
                        t += V[i][k]*V[i][j];
                    }
                    t = -t/V[k+1][k];
                    for (int i = k+1; i < n; i++) {
                        V[i][j] += t*V[i][k];
                    }
                }
            }
            for (int i = 0; i < n; i++) {
                V[i][k] = 0.0;
            }
            V[k][k] = 1.0;
        }
    }


    // Main iteration loop for the singular values.
    int pp = p-1;
    int iter = 0;
    double eps = pow(2.0,-52.0);
    double tiny = pow(2.0,-966.0);
    while (p > 0) {
        int k,kase;

        // XXX Here is where a test for too many iterations would go.

        // This section of the program inspects for
        // negligible elements in the s and e arrays.  On
        // completion the variables kase and k are set as follows.

        // kase = 1     if s(p) and e[k-1] are negligible and k<p
        // kase = 2     if s(k) is negligible and k<p
        // kase = 3     if e[k-1] is negligible, k<p, and
        //              s(k), ..., s(p) are not negligible (qr step).
        // kase = 4     if e(p-1) is negligible (convergence).

        for (k = p-2; k >= -1; k--) {
            if (k == -1) {
                break;
            }
            if (fabs(e[k]) <=
                tiny + eps*(fabs(s[k]) + fabs(s[k+1]))) {
                e[k] = 0.0;
                break;
            }
        }
        if (k == p-2) {
            kase = 4;
        } else {
            int ks;
            for (ks = p-1; ks >= k; ks--) {
                if (ks == k) {
                    break;
                }
                double t = (ks != p ? fabs(e[ks]) : 0.) +
                    (ks != k+1 ? fabs(e[ks-1]) : 0.);
                if (fabs(s[ks]) <= tiny + eps*t)  {
                    s[ks] = 0.0;
                    break;
                }
            }
            if (ks == k) {
                kase = 3;
            } else if (ks == p-1) {
                kase = 1;
            } else {
                kase = 2;
                k = ks;
            }
        }
        k++;

        // Perform the task indicated by kase.

        switch (kase) {

            // Deflate negligible s(p).

            case 1: {
                double f = e[p-2];
                e[p-2] = 0.0;
                for (int j = p-2; j >= k; j--) {
                    double t = c_hypot(s[j],f);
                    double cs = s[j]/t;
                    double sn = f/t;
                    s[j] = t;
                    if (j != k) {
                        f = -sn*e[j-1];
                        e[j-1] = cs*e[j-1];
                    }
                    if (wantv) {
                        for (int i = 0; i < n; i++) {
                            t = cs*V[i][j] + sn*V[i][p-1];
                            V[i][p-1] = -sn*V[i][j] + cs*V[i][p-1];
                            V[i][j] = t;
                        }
                    }
                }
            }
                break;

                // Split at negligible s(k).

            case 2: {
                double f = e[k-1];
                e[k-1] = 0.0;
                for (int j = k; j < p; j++) {
                    double t = c_hypot(s[j],f);
                    double cs = s[j]/t;
                    double sn = f/t;
                    s[j] = t;
                    f = -sn*e[j];
                    e[j] = cs*e[j];
                    if (wantu) {
                        for (int i = 0; i < m; i++) {
                            t = cs*U[i][j] + sn*U[i][k-1];
                            U[i][k-1] = -sn*U[i][j] + cs*U[i][k-1];
                            U[i][j] = t;
                        }
                    }
                }
            }
                break;

                // Perform one qr step.

            case 3: {

                // Calculate the shift.

                double scale = GET_MAX(GET_MAX(GET_MAX(GET_MAX(
                                                           fabs(s[p-1])
                                                           ,fabs(s[p-2]))
                                                       ,fabs(e[p-2])),
                                               fabs(s[k])),
                                       fabs(e[k]));
                double sp = s[p-1]/scale;
                double spm1 = s[p-2]/scale;
                double epm1 = e[p-2]/scale;
                double sk = s[k]/scale;
                double ek = e[k]/scale;
                double b = ((spm1 + sp)*(spm1 - sp) + epm1*epm1)/2.0;
                double c = (sp*epm1)*(sp*epm1);
                double shift = 0.0;
                if ((b != 0.0) | (c != 0.0)) {
                    shift = sqrt(b*b + c);
                    if (b < 0.0) {
                        shift = -shift;
                    }
                    shift = c/(b + shift);
                }
                double f = (sk + sp)*(sk - sp) + shift;
                double g = sk*ek;

                // Chase zeros.

                for (int j = k; j < p-1; j++) {
                    double t = c_hypot(f,g);
                    double cs = f/t;
                    double sn = g/t;
                    if (j != k) {
                        e[j-1] = t;
                    }
                    f = cs*s[j] + sn*e[j];
                    e[j] = cs*e[j] - sn*s[j];
                    g = sn*s[j+1];
                    s[j+1] = cs*s[j+1];
                    if (wantv) {
                        for (int i = 0; i < n; i++) {
                            t = cs*V[i][j] + sn*V[i][j+1];
                            V[i][j+1] = -sn*V[i][j] + cs*V[i][j+1];
                            V[i][j] = t;
                        }
                    }
                    t = c_hypot(f,g);
                    cs = f/t;
                    sn = g/t;
                    s[j] = t;
                    f = cs*e[j] + sn*s[j+1];
                    s[j+1] = -sn*e[j] + cs*s[j+1];
                    g = sn*e[j+1];
                    e[j+1] = cs*e[j+1];
                    if (wantu && (j < m-1)) {
                        for (int i = 0; i < m; i++) {
                            t = cs*U[i][j] + sn*U[i][j+1];
                            U[i][j+1] = -sn*U[i][j] + cs*U[i][j+1];
                            U[i][j] = t;
                        }
                    }
                }
                e[p-2] = f;
                iter = iter + 1;
            }
                break;

                // Convergence.

            case 4: {

                // Make the singular values positive.

                if (s[k] <= 0.0) {
                    s[k] = (s[k] < 0.0 ? -s[k] : 0.0);
                    if (wantv) {
                        for (int i = 0; i <= pp; i++) {
                            V[i][k] = -V[i][k];
                        }
                    }
                }

                // Order the singular values.

                while (k < pp) {
                    if (s[k] >= s[k+1]) {
                        break;
                    }
                    double t = s[k];
                    s[k] = s[k+1];
                    s[k+1] = t;
                    if (wantv && (k < n-1)) {
                        for (int i = 0; i < n; i++) {
                            t = V[i][k+1]; V[i][k+1] = V[i][k]; V[i][k] = t;
                        }
                    }
                    if (wantu && (k < m-1)) {
                        for (int i = 0; i < m; i++) {
                            t = U[i][k+1]; U[i][k+1] = U[i][k]; U[i][k] = t;
                        }
                    }
                    k++;
                }
                iter = 0;
                p--;
            }
                break;
        }
    }

    matd_t* V_lastCol = matd_create(4,1);
    for(int i=0; i<m; i++){
        matd_put(V_lastCol,i,0,V[i][3]);
    }
    return V_lastCol;
}

matd_t * c_triangulate(matd_t* Pleft, matd_t* Pright, double pxleft[2], double pxright[2])
{
    matd_t* p1t_left = matd_create(1,4);
    matd_t* p2t_left = matd_create(1,4);
    matd_t* p3t_left = matd_create(1,4);
    matd_t* p1t_right = matd_create(1,4);
    matd_t* p2t_right = matd_create(1,4);
    matd_t* p3t_right = matd_create(1,4);

    for(int i=0; i<4; i++){
        matd_put(p1t_left,0,i,matd_get(Pleft,0,i));
        matd_put(p2t_left,0,i,matd_get(Pleft,1,i));
        matd_put(p3t_left,0,i,matd_get(Pleft,2,i));

        matd_put(p1t_right,0,i,matd_get(Pright,0,i));
        matd_put(p2t_right,0,i,matd_get(Pright,1,i));
        matd_put(p3t_right,0,i,matd_get(Pright,2,i));
    }

    matd_t* A = matd_create(4,4);
    matd_t* tmp = matd_scale(p3t_left,pxleft[0]);
    matd_t* tmp2 = matd_op("M-M",tmp,p1t_left);
    for(int i=0;i<4;i++){
        matd_put(A,0,i,matd_get(tmp2,0,i));
    }
    matd_destroy(tmp);
    matd_destroy(tmp2);

    tmp = matd_scale(p3t_left,pxleft[1]);
    tmp2 = matd_op("M-M",tmp,p2t_left);
    for(int i=0;i<4;i++){
        matd_put(A,1,i,matd_get(tmp2,0,i));
    }
    matd_destroy(tmp);
    matd_destroy(tmp2);

    tmp = matd_scale(p3t_right,pxright[0]);
    tmp2 = matd_op("M-M",tmp,p1t_right);
    for(int i=0;i<4;i++){
        matd_put(A,2,i,matd_get(tmp2,0,i));
    }
    matd_destroy(tmp);
    matd_destroy(tmp2);

    tmp = matd_scale(p3t_right,pxright[1]);
    tmp2 = matd_op("M-M",tmp,p2t_right);
    for(int i=0;i<4;i++){
        matd_put(A,3,i,matd_get(tmp2,0,i));
    }

    matd_destroy(tmp);
    matd_destroy(tmp2);


    matd_t* V_lastCol = SVD_get_V(A);
    double lastElement = 1.0/matd_get(V_lastCol,3,0);
    matd_t* V_lastCol_scaled = matd_scale(V_lastCol,lastElement);

    matd_t* returnMatd = matd_create(3,1);
    matd_put(returnMatd,0,0,matd_get(V_lastCol_scaled,0,0));
    matd_put(returnMatd,1,0,matd_get(V_lastCol_scaled,1,0));
    matd_put(returnMatd,2,0,matd_get(V_lastCol_scaled,2,0));
    matd_destroy(A);
    matd_destroy(V_lastCol);
    matd_destroy(V_lastCol_scaled);
    matd_destroy(p1t_left);
    matd_destroy(p2t_left);
    matd_destroy(p3t_left);
    matd_destroy(p1t_right);
    matd_destroy(p2t_right);
    matd_destroy(p3t_right);

    return returnMatd;

}

matd_t * pinhole_transform(const matd_t *T, const matd_t *p)
{
    const double *Td = T->data;
    const double *pd = p->data;

    int ntcols = T->ncols;

    matd_t *r = NULL;

    if (T->nrows == 3 && T->ncols == 3) {
        if (p->nrows == 2 && p->ncols == 1) {
            r = matd_create(2, 1);
            double a = Td[0*ntcols+0]*pd[0] + Td[0*ntcols+1]*pd[1] + Td[0*ntcols+2];
            double b = Td[1*ntcols+0]*pd[0] + Td[1*ntcols+1]*pd[1] + Td[1*ntcols+2];
            double c = Td[2*ntcols+0]*pd[0] + Td[2*ntcols+1]*pd[1] + Td[2*ntcols+2];
            r->data[0] = a/c;
            r->data[1] = b/c;
        }

        if (p->nrows == 3 && p->ncols == 1) {
            r = matd_create(2, 1);
            double a = Td[0*ntcols+0]*pd[0] + Td[0*ntcols+1]*pd[1] + Td[0*ntcols+2]*pd[2];
            double b = Td[1*ntcols+0]*pd[0] + Td[1*ntcols+1]*pd[1] + Td[1*ntcols+2]*pd[2];
            double c = Td[2*ntcols+0]*pd[0] + Td[2*ntcols+1]*pd[1] + Td[2*ntcols+2]*pd[2];
            r->data[0] = a/c;
            r->data[1] = b/c;
        }
    }

    if (r == NULL) {
        fprintf(stderr, "camera_math pinhole_transform: Unsupported dimensions: T (%d, %d) p (%d, %d)\n",
                T->nrows, T->ncols, p->nrows, p->ncols);
        assert(0);
    }

    return r;
}

matd_t * point_transform(const matd_t *T, const matd_t *p)
{
    const double *Td = T->data;
    const double *pd = p->data;

    int ntcols = T->ncols;

    matd_t *r = NULL;

    if (T->nrows == 3 && T->ncols == 3) {
        if (p->nrows == 3 && p->ncols == 1) {
            r = matd_create(3, 1);
            r->data[0] = Td[0*ntcols+0]*pd[0] + Td[0*ntcols+1]*pd[1] + Td[0*ntcols+2]*pd[2];
            r->data[1] = Td[1*ntcols+0]*pd[0] + Td[1*ntcols+1]*pd[1] + Td[1*ntcols+2]*pd[2];
            r->data[2] = Td[2*ntcols+0]*pd[0] + Td[2*ntcols+1]*pd[1] + Td[2*ntcols+2]*pd[2];
        }
    }

    if (T->nrows == 4 && T->ncols == 4) {
        if (p->nrows == 3 && p->ncols == 1) {
            r = matd_create(3, 1);
            r->data[0] = Td[0*ntcols+0]*pd[0] + Td[0*ntcols+1]*pd[1] + Td[0*ntcols+2]*pd[2] + Td[0*ntcols+3];
            r->data[1] = Td[1*ntcols+0]*pd[0] + Td[1*ntcols+1]*pd[1] + Td[1*ntcols+2]*pd[2] + Td[1*ntcols+3];
            r->data[2] = Td[2*ntcols+0]*pd[0] + Td[2*ntcols+1]*pd[1] + Td[2*ntcols+2]*pd[2] + Td[2*ntcols+3];
        }
    }

    if (r == NULL) {
        fprintf(stderr, "camera_math point_transform: Unsupported dimensions: T (%d, %d) p (%d, %d)\n",
                T->nrows, T->ncols, p->nrows, p->ncols);
        assert(0);
    }

    return r;
}

matd_t * snap_ray_to_plane(const matd_t *p)
{
    matd_t *r = NULL;

    // x and y already normalized, z == 1 plane
    if (p->nrows == 2 && p->ncols == 1) {
        r = matd_create(3, 1);
        r->data[0] = p->data[0];
        r->data[1] = p->data[1];
        r->data[2] = 1;
        return r;
    }

    // normalize x and y, set z == 1
    if (p->nrows == 3 && p->ncols == 1) {
        r = matd_create(3, 1);
        r->data[0] = p->data[0] / p->data[2];
        r->data[1] = p->data[1] / p->data[2];
        r->data[2] = 1;
        return r;
    }

    if (r == NULL) {
        fprintf(stderr, "camera_math: snap_ray_to_plane: p has unsupported dimensions (%d, %d)\n",
                p->nrows, p->ncols);
        exit(1);
    }

    return r;
}

matd_t * snap_ray_to_sphere(const matd_t *p)
{
    matd_t *r = NULL;

    // assume point was on z == 1 plane
    if (p->nrows == 2 && p->ncols == 1) {
        r = matd_create(3, 1);
        r->data[0] = p->data[0];
        r->data[1] = p->data[1];
        r->data[2] = 1;
    }

    // unit magnitude sphere
    if (p->nrows == 3 && p->ncols == 1) {
        r = matd_create(3, 1);
        r->data[0] = p->data[0];
        r->data[1] = p->data[1];
        r->data[2] = p->data[2];
    }

    if (r == NULL) {
        fprintf(stderr, "camera_math: snap_ray_to_plane: p has unsupported dimensions (%d, %d)\n",
                p->nrows, p->ncols);
        exit(1);
    }

    matd_t * r_norm = matd_vec_normalize(r);
    matd_destroy(r);

    return r_norm;
}

matd_t * make_camera_matrix(const matd_t *K, const matd_t *B2C)
{
    assert(K->nrows == 3);
    assert(K->ncols == 3);
    assert(B2C->nrows == 3 || B2C->nrows == 4);
    assert(B2C->ncols == 4);

    matd_t *r = matd_create(3, 4);

    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 4; col++) {

            double acc = 0;
            for (int i = 0; i < 3; i++)
                //acc += K->data[i]*B2C->data[i];
                acc += matd_get(K,row,i)*matd_get(B2C,i,col);

            //r->data[row*4+col] = acc;
            matd_put(r,row,col,acc);
        }
    }

    return r;
}

zarray_t * project_intrin_list(const matd_t *K, const matd_t *B2C, zarray_t *xyzs)
{
    zarray_t *r = zarray_create(sizeof(matd_t*));

    for (int i = 0; i < zarray_size(xyzs); i++) {
        matd_t *xyz = NULL;
        zarray_get(xyzs, i, &xyz);

        matd_t *xy_dp = project_intrin(K, B2C, xyz);
        zarray_add(r, &xy_dp);
    }

    return r;
}

matd_t * project_intrin(const matd_t *K, const matd_t *B2C, const matd_t *xyz)
{
    assert(K->nrows == 3);
    assert(K->ncols == 3);
    assert(B2C->nrows == 3 || B2C->nrows == 4);
    assert(B2C->ncols == 4);

    if (B2C == NULL)
        return pinhole_transform(K, xyz);

    matd_t *xyz_camera = point_transform(B2C, xyz);
    matd_t *xy = pinhole_transform(K, xyz_camera);
    matd_destroy(xyz_camera);

    return xy;
}

matd_t * image_to_vis_transform(int imwidth, int imheight, double XY0[2], double XY1[2], int _flip)
{
    // create matrices
    matd_t *trans = matd_create(4, 4);
    matd_t *scale = matd_create(4, 4);
    matd_t *flip  = matd_create(4, 4);

    // make identity
    for (int i = 0; i < 4; i++) trans->data[i*4+i] = 1;
    for (int i = 0; i < 4; i++) scale->data[i*4+i] = 1;
    for (int i = 0; i < 4; i++) flip->data[i*4+i]  = 1;

    // set matrices
    trans->data[0*4+3] = XY0[0];
    trans->data[1*4+3] = XY0[1];

    scale->data[0*4+0] = (XY1[0] - XY0[0])/imwidth;
    scale->data[1*4+1] = (XY1[1] - XY0[1])/imheight;

    if (_flip) {
        flip->data[1*4+3] = imheight;
        flip->data[1*4+1] = -1;
        flip->data[2*4+2] = -1;
    }

    // multiply matrices
    matd_t *T = matd_op("MMM", trans, scale, flip);

    matd_destroy(trans);
    matd_destroy(scale);
    matd_destroy(flip);

    return T;
}

matd_t *camera_math_estimate_homography(const zarray_t *xys,
                                        const zarray_t *xy_primes)
{
    int npoints = zarray_size(xys);
    assert(npoints == zarray_size(xy_primes));

    matd_t *T = camera_math_compute_normalizing_transform(xys);
    matd_t *Tprime = camera_math_compute_normalizing_transform(xy_primes);
    //printf("T:\n"); matd_print(T, "%12.6f");
    //printf("Tprime:\n"); matd_print(Tprime, "%12.6f");

    matd_t *A = matd_create(2*npoints, 9);

    for (int i = 0; i < npoints; i++)
    {
        matd_t *xy = NULL, *xy_prime = NULL;
        zarray_get(xys, i, &xy);
        zarray_get(xy_primes, i, &xy_prime);

        matd_t *xy_norm = pinhole_transform(T, xy);
        matd_t *xy_prime_norm = pinhole_transform(Tprime, xy_prime);

        double x = xy_norm->data[0];
        double y = xy_norm->data[1];
        double w = 1;
        double xp = xy_prime_norm->data[0];
        double yp = xy_prime_norm->data[1];
        double wp = 1;

        double *A0 = &A->data[(2*i+0)*A->ncols];
        A0[0] =     0; A0[1] =     0; A0[2] =     0;
        A0[3] = -wp*x; A0[4] = -wp*y; A0[5] = -wp*w;
        A0[6] =  yp*x; A0[7] =  yp*y; A0[8] =  yp*w;

        double *A1 = &A->data[(2*i+1)*A->ncols];
        A1[0] =  wp*x; A1[1] =  wp*y; A1[2] =  wp*w;
        A1[3] =     0; A1[4] =     0; A1[5] =     0;
        A1[6] = -xp*x; A1[7] = -xp*y; A1[8] = -xp*w;

        matd_destroy(xy_norm);
        matd_destroy(xy_prime_norm);
    }

    //printf("A:\n");matd_print(A, "%12.6f");

    matd_svd_t svd = matd_svd(A);
    //printf("U:\n");matd_print(svd.U, "%12.6f");
    //printf("S:\n");matd_print(svd.S, "%12.6f");
    //printf("V:\n");matd_print(svd.V, "%12.6f");

    // make homography from last column
    matd_t *Ht = matd_create(3,3);
    for (int i = 0; i < 9; i++) {
        Ht->data[i] = svd.V->data[i*svd.V->ncols + (svd.V->ncols-1)];
    }
    //printf("Ht:\n");matd_print(Ht, "%12.6f");

    // undo normalization transforms
    matd_t *H = matd_op("(M^-1)MM", Tprime, Ht, T);
    //printf("H:\n");matd_print(H, "%12.6f");

    matd_destroy(T);
    matd_destroy(Tprime);
    matd_destroy(A);
    matd_destroy(svd.U);
    matd_destroy(svd.S);
    matd_destroy(svd.V);
    matd_destroy(Ht);

    return H;
}

// store { xmean, ymean, rms distance } in xyr
static void compute_centroid_and_rms(const zarray_t *xys, double xyr[3])
{
    int npoints = zarray_size(xys);
    double xsum = 0, ysum = 0;

    for (int i = 0; i < npoints; i++) {
        matd_t *xy = NULL;
        zarray_get(xys, i, &xy);

        xsum += xy->data[0];
        ysum += xy->data[1];
    }

    double xmean = xsum / npoints;
    double ymean = ysum / npoints;

    double acc = 0;
    for (int i = 0; i < npoints; i++) {
        matd_t *xy = NULL;
        zarray_get(xys, i, &xy);

        double dx = xy->data[0] - xmean;
        double dy = xy->data[1] - ymean;
        double sqdist = dx*dx + dy*dy;

        acc += sqdist;
    }

    double rms = sqrt(acc/npoints);

    xyr[0] = xmean;
    xyr[1] = ymean;
    xyr[2] = rms;
}

matd_t *camera_math_compute_normalizing_transform(const zarray_t *xys)

{
    double xyr[3] = { 0, 0, 0 };
    compute_centroid_and_rms(xys, xyr);

    double scale = sqrt(2) / xyr[2];

    matd_t *T = matd_create(3,3);
#define T(i,j) (T->data[i*T->ncols+j])
    T(0,0) = scale;
    T(1,1) = scale;
    T(0,2) = -xyr[0]*scale;
    T(1,2) = -xyr[1]*scale;
    T(2,2) = 1;
#undef T

    return T;
}

matd_t *camera_math_decompose_homography(const matd_t *H, const matd_t *K,
                                         const matd_t *p)
{
    // H is typically equal to K*Rt (for a 3x3 matrix Rt). However, H is only
    // ever known up to scale, so we must expect H=S*K*Rt Conveniently,
    // H = S*K*Rt = K*S*Rt, which means we can premultiply by inv(K) to
    // recover the scale rotation component
    matd_t *SRt = matd_op("(M^-1)M", K, H);

    // compute the scale, as the columns of R in Rt=[R_3x2 | t_3x1]
    // should have unit magnitude
#define SRt(i,j) (SRt->data[(i)*SRt->ncols+(j)])
    double s0 = sqrt(SRt(0,0)*SRt(0,0) + SRt(1,0)*SRt(1,0) + SRt(2,0)*SRt(2,0));
    double s1 = sqrt(SRt(0,1)*SRt(0,1) + SRt(1,1)*SRt(1,1) + SRt(2,1)*SRt(2,1));
    double scale = sqrt(s0*s1);
#undef SRt

    matd_t *Rt = matd_scale(SRt, 1.0/scale);

#define Rt(i,j) (Rt->data[(i)*Rt->ncols+(j)])
    double _T[] = { Rt(0,0), Rt(0,1),     0.0, Rt(0,2),
                    Rt(1,0), Rt(1,1),     0.0, Rt(1,2),
                    Rt(2,0), Rt(2,1),     0.0, Rt(2,2),
                        0.0,     0.0,     0.0,     1.0 };
    matd_t *T = matd_create_data(4,4,_T);
#undef Rt

    matd_t *pp = matd_transform(T, p);

    // pp must be in front of the camera (positive z)
    if (pp->data[2] < 0) {
        matd_scale_inplace(T, -1);
    }

#define T(i,j) (T->data[(i)*T->ncols+j])
    // last row could have changed in flip operation
    T(3,0) = 0; T(3,1) = 0; T(3,2) = 0; T(3,3) = 1;

    // recover third rotation vector by cross product
    matd_t *a = matd_create(3,1);
    a->data[0] = T(0,0); a->data[1] = T(1,0); a->data[2] = T(2,0);

    matd_t *b = matd_create(3,1);
    b->data[0] = T(0,1); b->data[1] = T(1,1); b->data[2] = T(2,1);

    matd_t *ab = matd_crossproduct(a, b);

    T(0,2) = ab->data[0]; T(1,2) = ab->data[1]; T(2,2) = ab->data[2];

    // pull out just the rotation component so we can normalize it.
    matd_t *R = matd_select(T, 0, 2, 0, 2);

    matd_svd_t svd = matd_svd(R);

    // polar decomposition, R = (UV')(VSV')
    matd_t *MR = matd_op("MM'", svd.U, svd.V);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            T->data[i*T->ncols+j] = MR->data[i*MR->ncols+j];
        }
    }
#undef T

    matd_destroy(SRt);
    matd_destroy(Rt);
    matd_destroy(a);
    matd_destroy(b);
    matd_destroy(ab);
    matd_destroy(R);
    matd_destroy(svd.U);
    matd_destroy(svd.S);
    matd_destroy(svd.V);

    return T;
}
