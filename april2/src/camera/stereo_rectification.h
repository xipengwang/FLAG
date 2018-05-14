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

#ifndef _STEREO_RECTIFICATION_H
#define _STEREO_RECTIFICATION_H

#include "common/matd.h"
#include "common/zarray.h"

#include "camera.h"
//#include "view.h"


typedef struct
{
    // inputs - temporary references
    const view_t * viewa;
    const view_t * viewb;
    const matd_t * B2Ca;
    const matd_t * B2Cb;

    // internal
    matd_t * K;
    matd_t * Ra_N2O;
    matd_t * Rb_N2O;

    // outputs
    srv_t *srva;
    srv_t *srvb;
    matd_t *B2Ca_new;
    matd_t *B2Cb_new;
}stereo_rect_t;


// max grown inscribed stereo rectified view
stereo_rect_t * stereo_rect_create_mgisrv(const view_t * viewa, const matd_t * B2Ca,
                                          const view_t * viewb, const matd_t * B2Cb);

// max inscribed stereo rectified view
stereo_rect_t * stereo_rect_create_misrv(const view_t * viewa, const matd_t * B2Ca,
                                         const view_t * viewb, const matd_t * B2Cb);

// max stereo rectified view
stereo_rect_t * stereo_rect_create_msrv(const view_t * viewa, const matd_t * B2Ca,
                                        const view_t * viewb, const matd_t * B2Cb);

void stereo_rect_destroy(stereo_rect_t * sr);

// return array of srv_ts
zarray_t * stereo_rect_get_views(const stereo_rect_t * sr);

// return array of matd_ts
zarray_t * stereo_rect_get_extrinsics_B2C(const stereo_rect_t * sr);

#endif
