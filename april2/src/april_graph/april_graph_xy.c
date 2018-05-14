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
// XY Factor
static april_graph_factor_t* xy_factor_copy(april_graph_factor_t *factor)
{
    april_graph_factor_t * next = calloc(1, sizeof(april_graph_factor_t));
    next->stype = factor->stype;
    next->type = factor->type;
    next->nnodes = factor->nnodes;
    next->nodes = calloc(1, sizeof(int) * next->nnodes);
    memcpy(next->nodes, factor->nodes, sizeof(int) * next->nnodes);
    next->length = factor->length;

    if (factor->attr) {
        assert(0);
    }
    next->copy = factor->copy;
    next->eval = factor->eval;
    next->destroy = factor->destroy;

    next->u.common.z = doubles_dup(factor->u.common.z, 2);
    next->u.common.ztruth = doubles_dup(factor->u.common.ztruth, 2);
    next->u.common.W = matd_copy(factor->u.common.W);

    return next;
}

static april_graph_factor_eval_t* xy_factor_eval(april_graph_factor_t *factor, april_graph_t *graph, april_graph_factor_eval_t *eval)
{
    april_graph_node_t *na, *nb;
    zarray_get(graph->nodes, factor->nodes[0], &na);
    zarray_get(graph->nodes, factor->nodes[1], &nb);
    assert(!(na->type==APRIL_GRAPH_NODE_XYT_TYPE &&
             nb->type==APRIL_GRAPH_NODE_XYT_TYPE));
    //XXX not very good code... hack for now
    if(na->type==APRIL_GRAPH_NODE_XYT_TYPE){
        //na is xyt node
        if (eval == NULL) {
            eval = calloc(1, sizeof(april_graph_factor_eval_t));
            eval->jacobians = calloc(3, sizeof(matd_t*)); // NB: NULL-terminated
            eval->jacobians[0] = matd_create(2,3);
            eval->jacobians[1] = matd_create(2,2);
            eval->r = calloc(2, sizeof(double));
            eval->W = matd_create(2,2);
        }

        double xa = na->state[0], ya = na->state[1];
        double xb = nb->state[0], yb = nb->state[1];

        // predicted obs
        double dx = xb - xa, dy = yb - ya;
        double zhat[2];
        zhat[0] = dx;
        zhat[1] = dy;

        // partial derivatives of zhat WRT node 0 [xa ya]
        matd_set_data(eval->jacobians[0], (double[]) {
                -1,  0, 0,
                    0, -1, 0});

        // partial derivatives of zhat WRT node 1 [xb yb]
        matd_set_data(eval->jacobians[1], (double[]) {
                1, 0,
                    0, 1});


        eval->length = 2;

        eval->r[0] = factor->u.common.z[0] - zhat[0];
        eval->r[1] = factor->u.common.z[1] - zhat[1];

        memcpy(eval->W->data, factor->u.common.W->data, 2*2*sizeof(double));

        // chi^2 = r'*W*r, via X = W*r
        double X[2] = { MATD_EL(eval->W, 0, 0)*eval->r[0] +
                        MATD_EL(eval->W, 0, 1)*eval->r[1],

                        MATD_EL(eval->W, 1, 0)*eval->r[0] +
                        MATD_EL(eval->W, 1, 1)*eval->r[1]};
        eval->chi2 = eval->r[0]*X[0] + eval->r[1]*X[1];

        return eval;

    }
    else if(nb->type==APRIL_GRAPH_NODE_XYT_TYPE){
        //nb is xyt node
        if (eval == NULL) {
            eval = calloc(1, sizeof(april_graph_factor_eval_t));
            eval->jacobians = calloc(3, sizeof(matd_t*)); // NB: NULL-terminated
            eval->jacobians[0] = matd_create(2,2);
            eval->jacobians[1] = matd_create(2,3);
            eval->r = calloc(2, sizeof(double));
            eval->W = matd_create(2,2);
        }

        double xa = na->state[0], ya = na->state[1];
        double xb = nb->state[0], yb = nb->state[1];

        // predicted obs
        double dx = xa - xb, dy = ya - yb;
        double zhat[2];
        zhat[0] = dx;
        zhat[1] = dy;

        // partial derivatives of zhat WRT node 0 [xa ya]
        matd_set_data(eval->jacobians[0], (double[]) {
                1,  0,
                    0, 1});

        // partial derivatives of zhat WRT node 1 [xb yb]
        matd_set_data(eval->jacobians[1], (double[]) {
                -1, 0, 0,
                    0, -1, 0});


        eval->length = 2;

        eval->r[0] = factor->u.common.z[0] - zhat[0];
        eval->r[1] = factor->u.common.z[1] - zhat[1];

        memcpy(eval->W->data, factor->u.common.W->data, 2*2*sizeof(double));

        // chi^2 = r'*W*r, via X = W*r
        double X[2] = { MATD_EL(eval->W, 0, 0)*eval->r[0] +
                        MATD_EL(eval->W, 0, 1)*eval->r[1],

                        MATD_EL(eval->W, 1, 0)*eval->r[0] +
                        MATD_EL(eval->W, 1, 1)*eval->r[1]};
        eval->chi2 = eval->r[0]*X[0] + eval->r[1]*X[1];

        return eval;

    }
    else{
        if (eval == NULL) {
            eval = calloc(1, sizeof(april_graph_factor_eval_t));
            eval->jacobians = calloc(3, sizeof(matd_t*)); // NB: NULL-terminated
            eval->jacobians[0] = matd_create(2,2);
            eval->jacobians[1] = matd_create(2,2);
            eval->r = calloc(2, sizeof(double));
            eval->W = matd_create(2,2);
        }

        double xa = na->state[0], ya = na->state[1];
        double xb = nb->state[0], yb = nb->state[1];

        // predicted obs
        double dx = xb - xa, dy = yb - ya;
        double zhat[2];
        zhat[0] = dx;
        zhat[1] = dy;

        // partial derivatives of zhat WRT node 0 [xa ya]
        matd_set_data(eval->jacobians[0], (double[]) {
                -1,  0,
                    0, -1});

        // partial derivatives of zhat WRT node 1 [xb yb]
        matd_set_data(eval->jacobians[1], (double[]) {
                1, 0,
                    0, 1});


        eval->length = 2;

        eval->r[0] = factor->u.common.z[0] - zhat[0];
        eval->r[1] = factor->u.common.z[1] - zhat[1];

        memcpy(eval->W->data, factor->u.common.W->data, 2*2*sizeof(double));

        // chi^2 = r'*W*r, via X = W*r
        double X[2] = { MATD_EL(eval->W, 0, 0)*eval->r[0] +
                        MATD_EL(eval->W, 0, 1)*eval->r[1],

                        MATD_EL(eval->W, 1, 0)*eval->r[0] +
                        MATD_EL(eval->W, 1, 1)*eval->r[1]};
        eval->chi2 = eval->r[0]*X[0] + eval->r[1]*X[1];

        return eval;
    }
}

static void xy_factor_destroy(april_graph_factor_t *factor)
{
    free(factor->nodes);
    free(factor->u.common.z);
    free(factor->u.common.ztruth);
    matd_destroy(factor->u.common.W);
    free(factor);
}

static void april_graph_factor_xy_encode(const stype_t *stype, uint8_t *data, uint32_t *datapos, const void *obj)
{
    const april_graph_factor_t *factor = obj;

    encode_u32(data, datapos, factor->nodes[0]);
    encode_u32(data, datapos, factor->nodes[1]);

    for (int i = 0; i < 2; i++)
        encode_f64(data, datapos, factor->u.common.z[i]);

    if (factor->u.common.ztruth) {
        encode_u8(data, datapos, 1);

        for (int i = 0; i < 2; i++)
            encode_f64(data, datapos, factor->u.common.ztruth[i]);
    } else {
        encode_u8(data, datapos, 0);
    }

    for (int i = 0; i < 2*2; i++)
        encode_f64(data, datapos, factor->u.common.W->data[i]);

    april_graph_attr_t *attr = factor->attr;
    stype_encode_object(data, datapos, attr ? attr->stype : NULL, attr);
}

static void *april_graph_factor_xy_decode(const stype_t *stype, const uint8_t *data, uint32_t *datapos, uint32_t datalen)
{
    int a = decode_u32(data, datapos, datalen);
    int b = decode_u32(data, datapos, datalen);

    double z[2];
    for (int i = 0; i < 2; i++)
        z[i] = decode_f64(data, datapos, datalen);

    double *ztruth = NULL;
    if (decode_u8(data, datapos, datalen)) {
        ztruth = malloc(2*sizeof(double));
        for (int i = 0; i < 2; i++)
            ztruth[i] = decode_f64(data, datapos, datalen);
    }

    matd_t *W = matd_create(2, 2);
    for (int i = 0; i < 2*2; i++)
        W->data[i] = decode_f64(data, datapos, datalen);

    april_graph_factor_t *factor = april_graph_factor_xy_create(a, b, z, ztruth, W);
    april_graph_attr_destroy(factor->attr);
    factor->attr = stype_decode_object(data, datapos, datalen, NULL);

    free(ztruth);
    matd_destroy(W);
    return factor;
}

const stype_t stype_april_factor_xy = { .name = "april_graph_factor_xy",
                                        .encode = april_graph_factor_xy_encode,
                                        .decode = april_graph_factor_xy_decode,
                                        .copy = NULL };

april_graph_factor_t *april_graph_factor_xy_create(int a, int b, const double *z, const double *ztruth, const matd_t *W)
{
    april_graph_factor_t *factor = calloc(1, sizeof(april_graph_factor_t));

    factor->stype = &stype_april_factor_xy;
    factor->type = APRIL_GRAPH_FACTOR_XY_TYPE;
    factor->nnodes = 2;
    factor->nodes = calloc(2, sizeof(int));
    factor->nodes[0] = a;
    factor->nodes[1] = b;
    factor->length = 2;

    factor->copy = xy_factor_copy;
    factor->eval = xy_factor_eval;
    factor->destroy = xy_factor_destroy;

    factor->u.common.z = doubles_dup(z, 2);
    factor->u.common.ztruth = doubles_dup(ztruth, 2);
    factor->u.common.W = matd_copy(W);

    return factor;
}

static void xy_node_update(april_graph_node_t *node, double *dstate)
{
    for (int i = 0; i < 2; i++)
        if(isnan(dstate[i])) return;
    for (int i = 0; i < 2; i++)
        node->state[i] += dstate[i];
}

static april_graph_node_t* xy_node_copy(april_graph_node_t *node)
{
    assert(0);
    return NULL;
}

static void xy_node_destroy(april_graph_node_t *node)
{
    free(node->state);
    free(node->init);
    free(node->truth);
    free(node);
}

static void april_graph_node_xy_encode(const stype_t *stype, uint8_t *data, uint32_t *datapos, const void *obj)
{
    const april_graph_node_t *node = obj;

    for (int i = 0; i < 2; i++)
        encode_f64(data, datapos, node->state[i]);

    if (node->init) {
        encode_u8(data, datapos, 1);
        for (int i = 0; i < 2; i++)
            encode_f64(data, datapos, node->init[i]);
    } else {
        encode_u8(data, datapos, 0);
    }

    if (node->truth) {
        encode_u8(data, datapos, 1);
        for (int i = 0; i < 2; i++)
            encode_f64(data, datapos, node->truth[i]);
    } else {
        encode_u8(data, datapos, 0);
    }

    april_graph_attr_t *attr = node->attr;
    stype_encode_object(data, datapos, attr ? attr->stype : NULL, attr);
}

static void *april_graph_node_xy_decode(const stype_t *stype, const uint8_t *data, uint32_t *datapos, uint32_t datalen)
{
    double state[2];

    for (int i = 0; i < 2; i++)
        state[i] = decode_f64(data, datapos, datalen);

    double *init = NULL;
    if (decode_u8(data, datapos, datalen)) {
        init = malloc(2*sizeof(double));
        for (int i = 0; i < 2; i++)
            init[i] = decode_f64(data, datapos, datalen);
    }

    double *truth = NULL;
    if (decode_u8(data, datapos, datalen)) {
        truth = malloc(2*sizeof(double));
        for (int i = 0; i < 2; i++)
            truth[i] = decode_f64(data, datapos, datalen);
    }

    april_graph_node_t *node = april_graph_node_xy_create(state, init, truth);
    april_graph_attr_destroy(node->attr);
    node->attr = stype_decode_object(data, datapos, datalen, NULL);

    free(init);
    free(truth);
    return node;
}

const stype_t stype_april_node_xy = { .name = "april_graph_node_xy",
                                      .encode = april_graph_node_xy_encode,
                                      .decode = april_graph_node_xy_decode,
                                      .copy = NULL };

april_graph_node_t *april_graph_node_xy_create(const double *state, const double *init, const double *truth)
{
    april_graph_node_t *node = calloc(1, sizeof(april_graph_node_t));
    node->type = APRIL_GRAPH_NODE_XY_TYPE;
    node->length = 2;
    node->state = doubles_dup(state, 2);
    node->init = doubles_dup(init, 2);
    node->truth = doubles_dup(truth, 2);

    node->update = xy_node_update;
    node->copy = xy_node_copy;
    node->destroy = xy_node_destroy;

    node->stype = &stype_april_node_xy;
    return node;
}

void april_graph_xy_stype_init()
{
    stype_register(&stype_april_node_xy);
    stype_register(&stype_april_factor_xy);
}
