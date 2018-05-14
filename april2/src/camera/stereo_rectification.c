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

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "common/matd.h"
#include "common/zarray.h"
#include "common/matd_coords.h"

#include "camera.h"
#include "view.h"
#include "max_rectified_view.h"
#include "max_inscribed_rectified_view.h"
#include "max_grown_inscribed_rectified_view.h"
#include "stereo_rectified_view.h"
#include "stereo_rectification.h"

#define EL(m, row,col) (m)->data[((row)*(m)->ncols + (col))]



////////////////////////////////////////////////////////////////////////////////
// HELPER FUNCTIONS

static stereo_rect_t * create_sr(const view_t * viewa, const matd_t * B2Ca,
                                 const view_t * viewb, const matd_t * B2Cb)
{
    stereo_rect_t *sr = calloc(1, sizeof(stereo_rect_t));
    sr->viewa = viewa;
    sr->viewb = viewb;
    sr->B2Ca = B2Ca;
    sr->B2Cb = B2Cb;
    return sr;
}

// clear these references because we don't actually own them (they aren't copied)
static void clear_sr_inputs(stereo_rect_t * sr)
{
    sr->viewa = NULL;
    sr->viewb = NULL;
    sr->B2Ca  = NULL;
    sr->B2Cb  = NULL;
}

static void compute_rbts(stereo_rect_t *sr)
{
    ////////////////////////////////////////
    // new intrinsics

    matd_t * Ka = sr->viewa->copy_intrinsics(sr->viewa);
    matd_t * Kb = sr->viewb->copy_intrinsics(sr->viewb);

    matd_t * K = matd_add(Ka, Kb);
    matd_scale_inplace(K, 0.5);
    // no skew
    K->data[0*K->ncols+1] = 0;

    matd_destroy(Ka);
    matd_destroy(Kb);

    sr->K = K;

    ////////////////////////////////////////
    // compute rotation

    zarray_t * fp = zarray_create(sizeof(matd_t*));

    matd_t * Ca2B = matd_inverse(sr->B2Ca); zarray_add(fp, &Ca2B);
    matd_t * Cb2B = matd_inverse(sr->B2Cb); zarray_add(fp, &Cb2B);

    double zeroarray[] = { 0, 0, 0 };
    matd_t * zero = matd_create_data(3, 1, zeroarray); zarray_add(fp, &zero);

    matd_t * c1 = matd_transform(Ca2B, zero); zarray_add(fp, &c1);
    matd_t * c2 = matd_transform(Cb2B, zero); zarray_add(fp, &c2);

    matd_t * vxraw = matd_subtract(c2, c1); zarray_add(fp, &vxraw);
    matd_t * vx    = matd_vec_normalize(vxraw); zarray_add(fp, &vx);

    // vy might not be right -- left camera is usually at the origin
    matd_t * Ca2Bz = matd_create(3, 1); zarray_add(fp, &Ca2Bz);
    EL(Ca2Bz,0,0) = EL(Ca2B,0,2);
    EL(Ca2Bz,0,1) = EL(Ca2B,1,2);
    EL(Ca2Bz,0,2) = EL(Ca2B,2,2);

    matd_t * vyraw = matd_crossproduct(Ca2Bz, vx); zarray_add(fp, &vyraw);
    matd_t * vy    = matd_vec_normalize(vyraw); zarray_add(fp, &vy);

    matd_t * vzraw = matd_crossproduct(vx, vy); zarray_add(fp, &vzraw);
    matd_t * vz    = matd_vec_normalize(vzraw); zarray_add(fp, &vz);

    double Ra_N2Oarray[] = { EL(vx,0,0), EL(vy,0,0), EL(vz,0,0), 0,
                             EL(vx,0,1), EL(vy,0,1), EL(vz,0,1), 0,
                             EL(vx,0,2), EL(vy,0,2), EL(vz,0,2), 0,
                             0,          0,          0,          1 };

    matd_t * Ra_N2O = matd_create_data(4,4, Ra_N2Oarray); zarray_add(fp, &Ra_N2O);

    ////////////////////////////////////////
    // new extrinsics

    // copy without translation
    matd_t * Ca2B_rot = matd_copy(Ca2B); zarray_add(fp, &Ca2B_rot);
    Ca2B_rot->data[0*Ca2B_rot->ncols+3] = 0;
    Ca2B_rot->data[1*Ca2B_rot->ncols+3] = 0;
    Ca2B_rot->data[2*Ca2B_rot->ncols+3] = 0;

    // copy without translation
    matd_t * Cb2B_rot = matd_copy(Cb2B); zarray_add(fp, &Cb2B_rot);
    Cb2B_rot->data[0*Cb2B_rot->ncols+3] = 0;
    Cb2B_rot->data[1*Cb2B_rot->ncols+3] = 0;
    Cb2B_rot->data[2*Cb2B_rot->ncols+3] = 0;

    matd_t * Ca2B_new = matd_op("MM", Ca2B, Ra_N2O); zarray_add(fp, &Ca2B_new);
    matd_t * Cb2B_new = matd_op("M(M^-1)MM", Cb2B, Cb2B_rot, Ca2B_rot, Ra_N2O); zarray_add(fp, &Cb2B_new);

    sr->B2Ca_new = matd_inverse(Ca2B_new);
    sr->B2Cb_new = matd_inverse(Cb2B_new);

    matd_t * Ra_N2O_final = matd_op("MM", sr->B2Ca, Ca2B_new); zarray_add(fp, &Ra_N2O_final);
    matd_t * Rb_N2O_final = matd_op("MM", sr->B2Cb, Cb2B_new); zarray_add(fp, &Rb_N2O_final);

    sr->Ra_N2O = matd_select(Ra_N2O_final, 0, 2, 0, 2);
    sr->Rb_N2O = matd_select(Rb_N2O_final, 0, 2, 0, 2);

    zarray_vmap(fp, matd_destroy);
    zarray_destroy(fp);
}

////////////////////////////////////////////////////////////////////////////////
// PUBLIC METHODS

void stereo_rect_destroy(stereo_rect_t * sr)
{
    // should have already been cleared
    assert(sr->viewa == NULL);
    assert(sr->viewb == NULL);
    assert(sr->B2Ca == NULL);
    assert(sr->B2Cb == NULL);

    matd_destroy(sr->K);
    matd_destroy(sr->Ra_N2O);
    matd_destroy(sr->Rb_N2O);

    srv_destroy(sr->srva);
    srv_destroy(sr->srvb);
    matd_destroy(sr->B2Ca_new);
    matd_destroy(sr->B2Cb_new);

    free(sr);
}

zarray_t * stereo_rect_get_views(const stereo_rect_t * sr)
{
    zarray_t * views = zarray_create(sizeof(srv_t*));

    zarray_add(views, &sr->srva);
    zarray_add(views, &sr->srvb);

    return views;
}

zarray_t * stereo_rect_get_extrinsics_B2C(const stereo_rect_t * sr)
{
    zarray_t * extrinsics = zarray_create(sizeof(matd_t*));

    zarray_add(extrinsics, &sr->B2Ca_new);
    zarray_add(extrinsics, &sr->B2Cb_new);

    return extrinsics;
}

////////////////////////////////////////////////////////////////////////////////
// MAX GROWN INSCRIBED STEREO RECTIFIED VIEW

stereo_rect_t * stereo_rect_create_mgisrv(const view_t * viewa, const matd_t * B2Ca,
                                          const view_t * viewb, const matd_t * B2Cb)
{
    stereo_rect_t *sr = create_sr(viewa, B2Ca, viewb, B2Cb);
    compute_rbts(sr);

    matd_t * KRa = matd_op("M(M^-1)", sr->K, sr->Ra_N2O);
    matd_t * KRb = matd_op("M(M^-1)", sr->K, sr->Rb_N2O);

    matd_t * xy01a = mgirv_compute_xy01(viewa, KRa);
    matd_t * xy01b = mgirv_compute_xy01(viewb, KRb);

    // make sure y offsets match. x offsets don't matter, but shared
    // y offsets ensure  that y indices in the image match
    EL(xy01a,0,1) = fmax(EL(xy01a,0,1), EL(xy01b,0,1));
    EL(xy01a,1,1) = fmin(EL(xy01a,1,1), EL(xy01b,1,1));
    EL(xy01b,0,1) = EL(xy01a,0,1);
    EL(xy01b,1,1) = EL(xy01a,1,1);

    sr->srva = srv_create(sr->K, xy01a);
    sr->srvb = srv_create(sr->K, xy01b);

    matd_destroy(xy01a);
    matd_destroy(xy01b);

    matd_destroy(KRa);
    matd_destroy(KRb);

    clear_sr_inputs(sr);
    return sr;
}

////////////////////////////////////////////////////////////////////////////////
// MAX INSCRIBED STEREO RECTIFIED VIEW

stereo_rect_t * stereo_rect_create_misrv(const view_t * viewa, const matd_t * B2Ca,
                                         const view_t * viewb, const matd_t * B2Cb)
{

    stereo_rect_t *sr = create_sr(viewa, B2Ca, viewb, B2Cb);
    compute_rbts(sr);

    matd_t * KRa = matd_op("M(M^-1)", sr->K, sr->Ra_N2O);
    matd_t * KRb = matd_op("M(M^-1)", sr->K, sr->Rb_N2O);

    matd_t * xy01a = mirv_compute_xy01(viewa, KRa);
    matd_t * xy01b = mirv_compute_xy01(viewb, KRb);

    // make sure y offsets match. x offsets don't matter, but shared
    // y offsets ensure  that y indices in the image match
    EL(xy01a,0,1) = fmax(EL(xy01a,0,1), EL(xy01b,0,1));
    EL(xy01a,1,1) = fmin(EL(xy01a,1,1), EL(xy01b,1,1));
    EL(xy01b,0,1) = EL(xy01a,0,1);
    EL(xy01b,1,1) = EL(xy01a,1,1);

    sr->srva = srv_create(sr->K, xy01a);
    sr->srvb = srv_create(sr->K, xy01b);

    matd_destroy(xy01a);
    matd_destroy(xy01b);

    matd_destroy(KRa);
    matd_destroy(KRb);

    clear_sr_inputs(sr);
    return sr;
}

////////////////////////////////////////////////////////////////////////////////
// MAX STEREO RECTIFIED VIEW

stereo_rect_t * stereo_rect_create_msrv(const view_t * viewa, const matd_t * B2Ca,
                                        const view_t * viewb, const matd_t * B2Cb)
{
    stereo_rect_t *sr = create_sr(viewa, B2Ca, viewb, B2Cb);
    compute_rbts(sr);

    matd_t * KRa = matd_op("M(M^-1)", sr->K, sr->Ra_N2O);
    matd_t * KRb = matd_op("M(M^-1)", sr->K, sr->Rb_N2O);

    matd_t * xy01a = mrv_compute_xy01(viewa, KRa);
    matd_t * xy01b = mrv_compute_xy01(viewb, KRb);

    // make sure y offsets match. x offsets don't matter, but shared
    // y offsets ensure  that y indices in the image match
    EL(xy01a,0,1) = fmax(EL(xy01a,0,1), EL(xy01b,0,1));
    EL(xy01a,1,1) = fmin(EL(xy01a,1,1), EL(xy01b,1,1));
    EL(xy01b,0,1) = EL(xy01a,0,1);
    EL(xy01b,1,1) = EL(xy01a,1,1);

    sr->srva = srv_create(sr->K, xy01a);
    sr->srvb = srv_create(sr->K, xy01b);

    matd_destroy(xy01a);
    matd_destroy(xy01b);

    matd_destroy(KRa);
    matd_destroy(KRb);

    clear_sr_inputs(sr);
    return sr;
}
