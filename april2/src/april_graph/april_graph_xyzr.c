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

#include "april_graph.h"
#include "common/doubles.h"
#include "common/matd.h"

/////////////////////////////////////////////////////////////////////////////////////////
// XYZR Factor
static april_graph_factor_t* xyzr_factor_copy(april_graph_factor_t *factor)
{
        assert(0);
        return NULL;

}

static april_graph_factor_eval_t* xyzr_factor_eval(april_graph_factor_t *factor, april_graph_t *graph, april_graph_factor_eval_t *eval)
{
    if (eval == NULL) {
        eval = calloc(1, sizeof(april_graph_factor_eval_t));
        eval->jacobians = calloc(3, sizeof(matd_t*)); // NB: NULL-terminated
        eval->jacobians[0] = matd_create(1,4);
        eval->jacobians[1] = matd_create(1,4);
        eval->r = calloc(1, sizeof(double));
        eval->W = matd_create(1,1);
    }

    april_graph_node_t *na, *nb;
    zarray_get(graph->nodes, factor->nodes[0], &na);
    zarray_get(graph->nodes, factor->nodes[1], &nb);

    double xa = na->state[0], ya = na->state[1], za = na->state[2], ta = na->state[3];
    double xb = nb->state[0], yb = nb->state[1], zb = nb->state[2], offset = nb->state[3];

    // predicted obs
    double zhat[1];

    double d = sqrt(sq(xa-xb) + sq(ya-yb) + sq(za-zb));
    zhat[0] =  d + offset;

    // partial derivatives of zhat WRT node 0 [xa ya za ta] robot position node
    matd_set_data(eval->jacobians[0], (double[]) {
            (xa-xb)/d, (ya-yb)/d, (za-zb)/d, 0});

    // partial derivatives of zhat WRT node 1 [xb yb zb offset] beacon position node
    matd_set_data(eval->jacobians[1], (double[]) {
            (xb-xa)/d, (yb-ya)/d, (zb-za)/d, 1});


    eval->length = 1;

    eval->r[0] = factor->u.common.z[0] - zhat[0];
    memcpy(eval->W->data, factor->u.common.W->data, 1*1*sizeof(double));

    // chi^2 = r'*W*r, via X = W*r
    double X = MATD_EL(eval->W, 0, 0)*eval->r[0];
    eval->chi2 = eval->r[0]*X;

    return eval;
}


static void xyzr_factor_destroy(april_graph_factor_t *factor)
{
    free(factor->nodes);
    free(factor->u.common.z);
    free(factor->u.common.ztruth);
    matd_destroy(factor->u.common.W);
    free(factor);
}



static void april_graph_factor_xyzr_encode(const stype_t *stype, uint8_t *data, uint32_t *datapos, const void *obj)
{
    assert(0);
}

static void *april_graph_factor_xyzr_decode(const stype_t *stype, const uint8_t *data, uint32_t *datapos, uint32_t datalen)
{
        assert(0);
        return NULL;

}

const stype_t stype_april_factor_xyzr = { .name = "april_graph_factor_xyzr",
                                         .encode = april_graph_factor_xyzr_encode,
                                         .decode = april_graph_factor_xyzr_decode,
                                         .copy = NULL };


april_graph_factor_t *april_graph_factor_xyzr_create(int a, int b, double *z, double *ztruth, matd_t *W)
{
    april_graph_factor_t *factor = calloc(1, sizeof(april_graph_factor_t));

    factor->type = APRIL_GRAPH_FACTOR_XYZR_TYPE;
    factor->nnodes = 2;
    factor->nodes = calloc(2, sizeof(int));
    factor->nodes[0] = a;
    factor->nodes[1] = b;
    factor->length = 1;

    factor->copy = xyzr_factor_copy;
    factor->eval = xyzr_factor_eval;
    factor->destroy = xyzr_factor_destroy;

    factor->u.common.z = doubles_dup(z, 1);
    factor->u.common.ztruth = doubles_dup(ztruth, 1);
    factor->u.common.W = matd_copy(W);

    return factor;
}


void april_graph_xyzr_stype_init()
{
    stype_register(&stype_april_factor_xyzr);
}
