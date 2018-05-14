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

#ifndef _SCANMATCH_H
#define _SCANMATCH_H

#include "common/zmaxheap.h"
#include "common/zarray.h"
#include "common/zhash.h"
#include "common/image_u8.h"


typedef struct sm_model sm_model_t;
struct sm_model
{
    // In units of meters_per_pixel, what is the coordinate of (the
    // outside corner) pixel 0? Note that lower-resolution versions have
    // slightly different origins. Specifically, index i is rounded
    // down (if necessary) to the next multiple of 2^i.
    int32_t x0, y0;

    image_u8_t *im;
};

typedef struct sm_model_data sm_model_data_t;
struct sm_model_data
{
    float meters_per_pixel; // resolution of highest-resolution model.

    // contains sm_model_t*, each entry successively
    // lower resolution (via max decimate)
    zarray_t *models;
    int uid;
};

///////////////////////////////////////////////////
struct sm_points_record
{
    float meters_per_pixel;
    float rad;
    int32_t level;
};

static uint32_t __attribute__ ((noinline)) sm_points_hasher(struct sm_points_record *rec)
{
    return rec->level*3274915.6717346 + rec->rad*237523.219871 + rec->meters_per_pixel*597111.68971;
}

#define TNAME sm_points_record_hash
#define TKEYTYPE struct sm_points_record
#define TVALTYPE zarray_t*
#define TKEYHASH(pk) sm_points_hasher(pk)
//#define TKEYHASH(pk) ((uint32_t) ((pk)->level*32749 + ((pk)->rad * 237) + ((pk)->meters_per_pixel*511)))
//#define TKEYEQUAL(pka, pkb) (!memcmp(pka, pkb, sizeof(struct sm_points_record)))
#define TKEYEQUAL(pka, pkb) ((pka)->level==(pkb)->level && (pka)->rad == (pkb)->rad && (pka)->meters_per_pixel == (pkb)->meters_per_pixel)
#include "common/thash_impl.h"
#undef TKEYEQUAL
#undef TKEYHASH
#undef TNAME
#undef TVALTYPE
#undef TKEYTYPE

typedef struct sm_points_data sm_points_data_t;
struct sm_points_data
{
    uint32_t flags;

    // contains float[2]; the original (non-quantized) data.
    zarray_t *points;

    // indexed by { (float) meters_per_pixel, (float) rad, (int32) level },
    // yields zarray_t* of int32_t[3] { ix, iy, count }.
    sm_points_record_hash_t *points_hash; // struct sm_points_record => zarray*<int32_t[3]>
    int uid;
};

struct sm_meaninf
{
    float rad_penalty;
    float u0, u1;
    float A00, A01, A11;
};

// This object is an opaque identifier for a specific call to
// sm_search_add, which allows searches to be aborted later on if
// desired. It also serves as a collection point for memory
// allocations specific to a particular search request.
typedef struct sm_search_handle sm_search_handle_t;
struct sm_search_handle
{
    sm_points_data_t *points_data;
    sm_model_data_t *model_data;

    // storage for the mean and information matrix data used for each
    // rotation. These are allocated all at once at _add time, and
    // individual sm_search_records contain a pointer into this.
    struct sm_meaninf *meaninfs;

    // used to make a copy of the original 3 variable information
    // matrix which is used for testing.
    float mean3[3];
    float inf33[9];

    float minscore_before_penalty;

    // the original bounds of the search. This is used to cull
    // children that fall outside the bounds.
    int tx0, tx1, ty0, ty1;

    float scale;
};

typedef struct sm_search_record sm_search_record_t;
struct sm_search_record
{
    sm_search_handle_t *handle;

    // meaninf stores the mean and information matrix conditioned on
    // rad.
    struct sm_meaninf *meaninf;

    // what resolution are we at?
    int32_t level;

    // blockx encodes a search range:
    // double tx0 = blockx * meters_per_pixel * 1<<level;
    // double ty0 = blocky * meters_per_pixel * 1<<level;
    int32_t blockx, blocky;

    // no rounding/bias effects apply to the rotation.
    float rad; // rotation
};

#define TNAME sm_search_record_heap
#define TVALTYPE struct sm_search_record
#include "common/tmaxheap_impl.h"
#undef TNAME
#undef TVALTYPE

typedef struct sm_search sm_search_t;
struct sm_search
{
    sm_search_record_heap_t *maxheap;
//    zmaxheap_t *maxheap; // a max heap

    // all the handles resulting from sm_search_add
    // that haven't yet been removed.
    zarray_t *handles;

};

typedef struct sm_result sm_result_t;
struct sm_result {
    sm_search_handle_t *handle;

    // note on biases:
    //
    // There MAY be a systematic bias in the translation values,
    // depending on how you construct the model. (A systematic bias
    // also results from the way in which points are decimated, but
    // this bias is already compensated-for when the result is
    // created.)
    //
    // *IF* your model was created with sub-pixel accurate models, then
    // your model has no inherent bias and no correction is required.
    //
    // *IF* your model was created by using coordinate truncation (via
    // *floor or "int" casts), you will need to had 0.5*meters_per_pixel
    // to the results.
    //
    // Additional discussion can be found in scanmatch.c
    double xyt[3];

    // score = scale * sum_i (score for point i) - penalty
    double score;
};

// create an object that can create (and cache) decimated and rotated
// point clouds.  The input parameter "points" will belong to the
// sm_points_data, and will be freed when the sm_points_data is
// destroyed.
// points is a float[2] (not a pointer)
sm_points_data_t *sm_points_data_create(zarray_t *points);

// NODECIMATE: don't decimate point cloud. There is no reason to turn this on, except
// for scientific measurement of the runtime improvement of the feature.
#define SM_POINTS_NODECIMATE 0
sm_points_data_t *sm_points_data_create_flags(zarray_t *points, uint32_t flags);

// we create rotated/decimated point clouds on-demand (and cache the
// results): no need to store level=0 copies of the original point
// cloud for every possible rotation when most rotations will be
// quickly ruled out (and the level 0 version never needed).
//
// create or return a cached version of the points subject to a
// rotation of "rad", and quantized according to
// (1<<level)*meters_per_pixel. The returned object should not be
// freed or modified; it belongs to the sm_points_data.
// (meters_per_pixel should correspond to the highest resolution.)
zarray_t *sm_points_data_get(sm_points_data_t *sm_data,
                             float meters_per_pixel, float rad, int level);

// destroys all point clouds that had been previously created and returned,
// and frees the "points" object originally passed in.
void sm_points_data_destroy(sm_points_data_t *sm_data);

void sm_points_data_clear_cache(sm_points_data_t *sm_data);

// the image becomes owned by the sm_model_data. Do not free it; it
// will be freed by sm_model_data_destroy
sm_model_data_t *sm_model_data_create(image_u8_t *im,
                                      int32_t x0, int32_t y0,
                                      float meters_per_pixel,
                                      int nresolutions);

// note that x0, y0 are given in INTEGERs. The meters_per_pixel isn't
// really used in this function: it's used during runtime to request a
// points object converted using the appropriate scale factor.
sm_model_data_t *sm_model_data_create_reference(image_u8_t *im,
                                                int32_t x0, int32_t y0,
                                                float meters_per_pixel, int nresolutions);

// it is an error to free a model that still belongs to search queries.
void sm_model_data_destroy(sm_model_data_t *model_data);


sm_search_t *sm_search_create();

void sm_search_destroy(sm_search_t *search);

// make a list of points corresponding to the non-zero pixels in the
// image. This is used by our testing infrastructure, but probably not
// super useful in most applications.
zarray_t *sm_image_to_points(image_u8_t *im);

zarray_t *sm_load_points_from_image(image_u8_t *im, float x0, float y0, float meters_per_pixel);

// add another multi-resolution search to the search engine, which
// will be interwoven with any other searches that have been added.
// Find a rigid-body transformation that projects the points to align
// with the model. Suppose A is the transform that projects the points
// into a global coordinate frame, and B is the transform that
// projects the model into the global coordinate frame.  Then T =
// inv(B)*A. (We are projecting into model B's coordinate frame; we
// first project points into the global coordinate frame by multiply
// by A, then project into model B's coordinate frame by multipling by
// inv(B).)
//
// tx0,tx1,ty0,ty1 represent the translation search range (in units of
// meters_per_pixel)
//
// rad0:radstep:rad1 encodes the search range for rotation (in
// radians). Note that down-sampled point clouds will be requested
// from the points_data object according to the rotations. It is more
// efficient if these rotations are equal to other rotations that have
// been requested before, so quantizing these can be beneficial.
//
// scale: the raw score (computed by summing the score for each point)
// is scaled by the factor 'scale'. It is conventional to use 1.0 / #points.
//
// mean,inf encode a prior penalty added to each alignment attempt. We
// subtract from the score the value: (x-mean)'*inf*(x-mean). The mean
// and information matrix (which is the inverse of the covariance
// matrix) should be scaled into units of meters_per_pixel.  mean
// should be a 3x1 [tx ty theta]. inf should be a 9 element array
// corresponding to the 3x3 (symmetric) matrix inv(covariance).
//
// minscore: do not retain any hypotheses whose score is less than
// this amount.  This can reduce the amount of time required to
// compute a "bad" answer, since the search will be terminated. Note,
// to guarantee *some* answer, this value should be -INF (since priors
// can cause scores to be negative).
//
// The return value should never be destroyed by the user; it will
// be deallocated by sm_search_destroy.
sm_search_handle_t *sm_search_add(sm_search_t *search,
                                  sm_points_data_t *points_data, sm_model_data_t *model_data,
                                  int32_t tx0, int32_t tx1, int32_t ty0, int32_t ty1,
                                  float rad0, float rad1, float radstep,
                                  float scale, float *mean, float *inf, float minscore);

// Stop any further expansion of search nodes related to this search. (Note:
// this function should only be called when sm_search_run has returned.)
void sm_search_remove(sm_search_t *search, sm_search_handle_t *handle);

void sm_search_record_destroy(sm_search_record_t *r);


// expand the heap until a maximum-resolution match is found.  Returns
// a match (which you must destroy with sm_result_destroy) or
// NULL, if no match was found.
sm_result_t *sm_search_run(sm_search_t *search);

void sm_result_destroy(sm_result_t *result);

////////////////////////////////////////////////////////////
typedef struct sm_hillclimb_result sm_hillclimb_result_t;
struct sm_hillclimb_result {
    double xyt[3];
    double score;   // total matching score minus penalty.
    double penalty; // the penalty (>=0) that was subtracted from score.
    int iters;
};

typedef struct sm_hillclimb_params sm_hillclimb_params_t;
struct sm_hillclimb_params {
    double initial_step_sizes[3]; // initial step size in x, y, theta
    double step_size_shrink_factor; // how much to shrink step size by. (e.g. 0.5)
    int max_step_size_shrinks; // how many times

    int maxiters;
};

// See the comments regarding biases in sm_search_result. The same
// biases apply to this algorithm. Note: hill climbing will create
// many requests in the sm_points_data. You may want to
// sm_points_data_clear_cache() after calling hillclimb.
sm_hillclimb_result_t *sm_hillclimb(sm_points_data_t *points_data, sm_model_data_t *model_data,
                                    double xyt0[3], sm_hillclimb_params_t *params,
                                    float scale, float *mean, float *inf);

void sm_hillclimb_result_destroy(sm_hillclimb_result_t *res);

////////////////////////////////////////////////////////////
typedef struct sm_icp_result sm_icp_result_t;
struct sm_icp_result
{
    double xyt[3];
    double weight;
    int iters;
};

typedef struct sm_icp_params sm_icp_params_t;
struct sm_icp_params {
    double max_dist_ratio;
    double trans_thresh_m;
    double rad_thresh;
    int    maxiters;
};

sm_icp_result_t sm_icp(zarray_t *pointsa, zarray_t *pointsb, double *xyt, sm_icp_params_t *params);

#endif
