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

#ifndef _APRIL_GRAPH_H
#define _APRIL_GRAPH_H

#include <stdlib.h>
#include "common/zarray.h"
#include "common/zhash.h"
#include "common/matd.h"
#include "common/stype.h"

/////////////////////////////////////////////////////////////
// april_graph_attr

struct april_graph_attr_record
{
    const stype_t *stype;
    void          *value;
};

typedef struct april_graph_attr april_graph_attr_t;
struct april_graph_attr
{
    zhash_t *hash;
    const stype_t *stype;
};

april_graph_attr_t *april_graph_attr_create();
void april_graph_attr_destroy(april_graph_attr_t *attr);

/////////////////////////////////////////////////////////////
// april_graph

typedef struct april_graph april_graph_t;
struct april_graph
{
    zarray_t *factors;
    zarray_t *nodes;

    april_graph_attr_t *attr;
    const stype_t *stype;
};

/////////////////////////////////////////////////////////////
// april_graph_factor_eval

typedef struct april_graph_factor_eval april_graph_factor_eval_t;
struct april_graph_factor_eval
{
    double chi2;

    // One jacobian for every node that this factor is connected to.
    // (order given by the factor's 'nodes' array). Should be NULL
    // terminated so that this structure can be deallocated without knowing
    // which factor created it.
    matd_t **jacobians;

    int length;
    double *r; // residual (length x 1)
    matd_t *W; // information matrix of observation (length x length)
};

#define APRIL_GRAPH_FACTOR_XYT_TYPE 1
#define APRIL_GRAPH_FACTOR_XYTPOS_TYPE 2
#define APRIL_GRAPH_FACTOR_MAX_TYPE 3
#define APRIL_GRAPH_FACTOR_XYPOS_TYPE 4
#define APRIL_GRAPH_FACTOR_R_FIXED_TYPE 5
#define APRIL_GRAPH_FACTOR_R_TYPE 6
#define APRIL_GRAPH_FACTOR_XYZT_TYPE 6
#define APRIL_GRAPH_FACTOR_XYZR_TYPE 7
#define APRIL_GRAPH_FACTOR_XYR_TYPE 8
#define APRIL_GRAPH_FACTOR_XYZPOS_TYPE 9
#define APRIL_GRAPH_FACTOR_C_TYPE 10
#define APRIL_GRAPH_FACTOR_BTF_TYPE 11
#define APRIL_GRAPH_FACTOR_B_TYPE 12
#define APRIL_GRAPH_FACTOR_R_OFFSET_TYPE 13
#define APRIL_GRAPH_FACTOR_XY_TYPE 14
#define APRIL_GRAPH_FACTOR_XYZ_TYPE 15
#define APRIL_GRAPH_FACTOR_RB_TYPE 16
#define APRIL_GRAPH_FACTOR_RB_OFFSET_TYPE 17
#define APRIL_GRAPH_FACTOR_BPOS_TYPE 18

#define APRIL_GRAPH_NODE_XYT_TYPE 100
#define APRIL_GRAPH_NODE_XY_TYPE 101
#define APRIL_GRAPH_NODE_XYZ_TYPE 102
#define APRIL_GRAPH_NODE_XYZT_TYPE 103
#define APRIL_GRAPH_NODE_BTF_TYPE 104
#define APRIL_GRAPH_NODE_B_TYPE 105

/////////////////////////////////////////////////////////////
// april_graph_factor

typedef struct april_graph_factor april_graph_factor_t;
struct april_graph_factor
{
    int type; // a unique identifier

    int nnodes;
    int *nodes;

    int length; // how many DOF?

    april_graph_attr_t *attr;

    april_graph_factor_t* (*copy)(april_graph_factor_t *factor);

    // evaluate this factor potential given the current graph. The
    // "eval" object can be expensive to build; for efficiency, a
    // caller can recycle an eval object previously constructed by
    // this object. If non-null, this object MUST be recycled and the
    // factor will return the passed-in "eval" object (with recomputed
    // numerical values). Otherwise, a new "eval" object is created
    // and returned.
    april_graph_factor_eval_t* (*eval)(april_graph_factor_t *factor, april_graph_t *graph, april_graph_factor_eval_t *eval);

    void (*destroy)(april_graph_factor_t *factor);

    // storage for implementations. They can use them however they
    // like.
    union {
        struct {
            double *z;
            double *ztruth;
            matd_t *W;
            void   *impl;
        } common;

        struct {
            april_graph_factor_t **factors;
            double *logw;
            int nfactors;
        } max;

        struct {
            void *impl;
        } impl;
    } u;

    const stype_t *stype;
};

/////////////////////////////////////////////////////////////
// april_graph_node

typedef struct april_graph_node april_graph_node_t;
struct april_graph_node
{
    int UID;
    int type;  // a unique identifier
    int length; // # of DOF

    double *state; // length x 1  vectors
    double *init;
    double *truth;

    april_graph_attr_t *attr;  // string (char*) to arbitrary pointer

    april_graph_node_t* (*copy)(april_graph_node_t *node);

    // called after some optimization; generically, should
    // implement state += dstate.
    void (*update)(april_graph_node_t *node, double *dstate);

    void (*destroy)(april_graph_node_t *node);

    void *impl;

    const stype_t *stype;
};

/////////////////////////////////////////////////////////////
// API

april_graph_t *april_graph_create();
april_graph_t *april_graph_create_from_file(const char *path);
void april_graph_destroy(april_graph_t *graph);

void april_graph_factor_eval_destroy(april_graph_factor_eval_t *eval);

typedef struct april_graph_gauss_seidel_info april_graph_gauss_seidel_info_t;
struct april_graph_gauss_seidel_info
{
    zarray_t *allneighbors;
    int nevals;
    april_graph_factor_eval_t **evals;
};

april_graph_gauss_seidel_info_t *april_graph_gauss_seidel_info_create(april_graph_t *graph);
void april_graph_gauss_seidel_info_destroy(april_graph_gauss_seidel_info_t *info);

void april_graph_gauss_seidel(april_graph_t *graph, april_graph_gauss_seidel_info_t *info);

typedef struct april_graph_cholesky_param april_graph_cholesky_param_t;
struct april_graph_cholesky_param
{
    // if non-zero, add tikahnov*I to the information matrix.
    double tikhanov;

    // Use the specified node ordering to reduce fill-in. If not
    // specified, an ordering is computed automatically.
    int *ordering;

    int show_timing;
};

// initialize to default values.
void april_graph_cholesky_param_init(april_graph_cholesky_param_t *param);

// Compute a Gauss-Newton update on the graph, using the specified
// node ordering. The ordering should specify the order for each
// april_graph_node_t; if NULL, a default ordering is computed. The
// ordering passed in belongs to the caller.
void april_graph_cholesky(april_graph_t *graph, april_graph_cholesky_param_t *param);

int april_graph_dof(april_graph_t *graph);
double april_graph_chi2(april_graph_t *graph);

void april_graph_postscript(april_graph_t *graph, const char *path);

// copies z, ztruth, W.
// copies state, init, truth. truth can be NULL.
// Z transforms points in coordinate frame B into coordinate frame A

april_graph_node_t *april_graph_node_xyt_create(const double *state, const double *init, const double *truth);
april_graph_node_t *april_graph_node_xy_create(const double *state, const double *init, const double *truth);
april_graph_node_t *april_graph_node_xyz_create(const double *state, const double *init, const double *truth);
april_graph_node_t *april_graph_node_xyzt_create(double *state, double *init, double *truth);
april_graph_node_t *april_graph_node_btf_create(double *state, double *init, double *truth);
april_graph_node_t *april_graph_node_b_create(double *state, double *init, double *truth);

april_graph_factor_t *april_graph_factor_xyt_create(int a, int b, const double *z, const double *ztruth, const matd_t *W);
april_graph_factor_t *april_graph_factor_xy_create(int a, int b, const double *z, const double *ztruth, const matd_t *W);
april_graph_factor_t *april_graph_factor_xyz_create(int a, int b, const double *z, const double *ztruth, const matd_t *W);
april_graph_factor_t *april_graph_factor_xyzt_create(int a, int b, double *z, double *ztruth, matd_t *W);
april_graph_factor_t *april_graph_factor_xyzr_create(int a, int b, double *z, double *ztruth, matd_t *W);
april_graph_factor_t *april_graph_factor_xyr_create(int a, int b, double *z, double *ztruth, matd_t *W);
april_graph_factor_t *april_graph_factor_r_fixed_create(int a, double *xy0, double *z, double *ztruth, matd_t *W);
april_graph_factor_t *april_graph_factor_xytpos_create(int a, double *z, double *ztruth, matd_t *W);
april_graph_factor_t *april_graph_factor_xypos_create(int a, double *z, double *ztruth, matd_t *W);
april_graph_factor_t *april_graph_factor_r_create(int a, int b, double *z, double *ztruth, matd_t *W);
april_graph_factor_t *april_graph_factor_r_offset_create(int a, int b, double *z, double *ztruth, matd_t *W, double *xyz_offset);
april_graph_factor_t *april_graph_factor_xyzpos_create(int a, double *z, double *ztruth, matd_t *W);
april_graph_factor_t *april_graph_factor_c_create(int a, int b, int c, int d, double *z, double *ztruth, matd_t *W);
april_graph_factor_t *april_graph_factor_btf_create(int a, int b, const double *z, const double *ztruth, const matd_t *W);
april_graph_factor_t *april_graph_factor_rb_create(int a, int b, int c, int d, double *z, double *ztruth, matd_t *W);
april_graph_factor_t *april_graph_factor_rb_offset_create(int a, int b, int c, int d, double *z, double *ztruth, matd_t *W, double *xyz_offset);
april_graph_factor_t *april_graph_factor_b_create(int a, int b, const double *z, const double *ztruth, const matd_t *W);
april_graph_factor_t *april_graph_factor_bpos_create(int a, const double *z, const double *ztruth, const matd_t *W);

// we take ownership of the factors and the array containing
// containing them. They will be freed when this factor is freed.
april_graph_factor_t *april_graph_factor_max_create(april_graph_factor_t **factors, double *logw, int nfactors);
april_graph_factor_t *april_graph_factor_max_best(april_graph_factor_t *factor, april_graph_t *graph);

void april_graph_attr_put(april_graph_t *graph, const stype_t *type, const char *key, void *data);
void* april_graph_attr_get(april_graph_t *graph, const char * key);

void april_graph_factor_attr_put(april_graph_factor_t *factor, const stype_t *type, const char *key, void *data);
void* april_graph_factor_attr_get(april_graph_factor_t *factor, const char * key);

void april_graph_node_attr_put(april_graph_node_t *node, const stype_t *type, const char *key, void *data);
void* april_graph_node_attr_get(april_graph_node_t *node, const char * key);

// Some predefined types
//XXX These name is quite long:
//XXX Taking advantage of non-portable sizeof(void*) == sizeof(int64_t) to surpress compiler
/*
const extern april_graph_type_t april_graph_type_int64;
const extern april_graph_type_t april_graph_type_str;
*/

/* XXX This method is experimental (and works poorly) **/
void april_graph_gradient_descent(april_graph_t *graph);

int april_graph_save(april_graph_t *graph, const char *path);

void april_graph_stype_init();

#endif
