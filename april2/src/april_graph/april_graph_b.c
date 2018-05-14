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
License along with this library; if not, see <http://www.gnu.org/licenses/>. */

#include "april_graph.h"
#include "common/doubles.h"
#include "common/matd.h"

#define FACTOR_LEN 1 // factor->length

/**
 * This file contains an implementation for an aprilgraph node that represents
 * an offset bias.
 */

///////////////////////////////////////////////////////
//               bpos factor impl                    //
///////////////////////////////////////////////////////

static april_graph_factor_t* bpos_factor_copy(april_graph_factor_t* factor)
{
    assert(0);
    return NULL;
}

static void bpos_factor_destroy(april_graph_factor_t* factor)
{
    free(factor->nodes);
    free(factor->u.common.z);
    free(factor->u.common.ztruth);
    matd_destroy(factor->u.common.W);
    free(factor);
}

static void april_graph_factor_bpos_encode(const stype_t *stype, uint8_t *data,
                                          uint32_t *datapos, const void *obj)
{
    const april_graph_factor_t *factor = obj;

    for (int i = 0; i < factor->nnodes; i++)
        encode_u32(data, datapos, factor->nodes[i]);

    for (int i = 0; i < factor->length; i++)
        encode_f64(data, datapos, factor->u.common.z[i]);

    if (factor->u.common.ztruth) {
        encode_u8(data, datapos, factor->length);
        for (int i = 0; i < factor->length; i++)
            encode_f64(data, datapos, factor->u.common.ztruth[i]);
    } else {
        encode_u8(data, datapos, 0);
    }

    for (int i = 0; i < factor->length; i++)
        encode_f64(data, datapos, factor->u.common.W->data[i]);

    april_graph_attr_t *attr = factor->attr;
    stype_encode_object(data, datapos, attr ? attr->stype : NULL, attr);
}

static void *april_graph_factor_bpos_decode(const stype_t *stype, const uint8_t *data, uint32_t *datapos, uint32_t datalen)
{
    int a = decode_u32(data, datapos, datalen);

    double z[FACTOR_LEN];
    for (int i = 0; i < FACTOR_LEN; i++)
        z[i] = decode_f64(data, datapos, datalen);

    double *ztruth = NULL;
    if (decode_u8(data, datapos, datalen)) {
        ztruth = malloc(FACTOR_LEN * sizeof(double));
        for (int i = 0; i < FACTOR_LEN; i++)
            ztruth[i] = decode_f64(data, datapos, datalen);
    }

    matd_t *W = matd_create(FACTOR_LEN, FACTOR_LEN);
    for (int i = 0; i < FACTOR_LEN; i++)
        W->data[i] = decode_f64(data, datapos, datalen);

    april_graph_factor_t *factor = april_graph_factor_bpos_create(a, z, ztruth, W);
    april_graph_attr_destroy(factor->attr);
    factor->attr = stype_decode_object(data, datapos, datalen, NULL);

    free(ztruth);
    matd_destroy(W);
    return factor;
}

const stype_t stype_april_factor_bpos = { .name = "april_graph_factor_bpos",
                                          .encode = april_graph_factor_bpos_encode,
                                          .decode = april_graph_factor_bpos_decode,
                                          .copy = NULL };

static april_graph_factor_eval_t* bpos_factor_eval(april_graph_factor_t* factor,
                                                   april_graph_t* graph,
                                                   april_graph_factor_eval_t *eval)
{
    april_graph_node_t* na;
    zarray_get(graph->nodes, factor->nodes[0], &na);
    assert(na->type == APRIL_GRAPH_NODE_B_TYPE);

    //Allocate a new eval if we weren't given one.
    //The first time this function is called for a given factor, a new eval
    //will have to be allocated, but after that the old eval is reused to save
    //on alloc/free costs.
    if(eval == NULL)
    {
        eval = calloc(1, sizeof(april_graph_factor_eval_t));

        eval->jacobians = calloc(factor->nnodes + 1, sizeof(matd_t*));    // NB: NULL-terminated
        eval->jacobians[0] = matd_create(factor->length, na->length);

        eval->length = factor->length;
        eval->r = calloc(eval->length, sizeof(double));
        eval->W = matd_create(factor->length, factor->length);
    }

    // partial derivatives of zhat WRT node 0 [b]
    MATD_EL(eval->jacobians[0], 0, 0) = 1;

    eval->length = factor->length;

    eval->r[0] = factor->u.common.z[0] - na->state[0];
    memcpy(eval->W->data, factor->u.common.W->data, factor->length * factor->length * sizeof(double));

    // chi^2 = r'*W*r, via X = W*r
    double X[] = { MATD_EL(eval->W, 0, 0)*eval->r[0] };
    eval->chi2 = eval->r[0]*X[0];

    return eval;
}

april_graph_factor_t *april_graph_factor_bpos_create(int a, const double *z,
                                                     const double *ztruth,
                                                     const matd_t *W)
{
    april_graph_factor_t *factor = calloc(1, sizeof(april_graph_factor_t));

    factor->stype = &stype_april_factor_bpos;
    factor->type = APRIL_GRAPH_FACTOR_BPOS_TYPE;
    factor->nnodes = 1;
    factor->nodes = calloc(factor->nnodes, sizeof(int));
    factor->nodes[0] = a;
    factor->length = 1;

    factor->copy = bpos_factor_copy;
    factor->eval = bpos_factor_eval;
    factor->destroy = bpos_factor_destroy;

    factor->u.common.z = doubles_dup(z, factor->length);
    factor->u.common.ztruth = doubles_dup(ztruth, factor->length);
    factor->u.common.W = matd_copy(W);

    return factor;
}

///////////////////////////////////////////////////////
//                  b factor impl                    //
///////////////////////////////////////////////////////

static april_graph_factor_t* b_factor_copy(april_graph_factor_t* factor)
{
    april_graph_factor_t* next = calloc(1, sizeof(april_graph_factor_t));
    next->stype = factor->stype;
    next->type = factor->type;
    next->nnodes = factor->nnodes;
    next->nodes = calloc(next->nnodes, sizeof(int));
    memcpy(next->nodes, factor->nodes, sizeof(int) * next->nnodes);
    next->length = factor->length;

    if(factor->attr)
    {
        assert(0);
    }
    next->copy = factor->copy;
    next->eval = factor->eval;
    next->destroy = factor->destroy;

    next->u.common.z = doubles_dup(factor->u.common.z, factor->length);
    next->u.common.ztruth = doubles_dup(factor->u.common.ztruth, factor->length);
    next->u.common.W = matd_copy(factor->u.common.W);

    return next;
}

static void b_factor_destroy(april_graph_factor_t* factor)
{
    free(factor->nodes);
    free(factor->u.common.z);
    free(factor->u.common.ztruth);
    matd_destroy(factor->u.common.W);
    free(factor);
}

static void april_graph_factor_b_encode(const stype_t *stype, uint8_t *data,
                                          uint32_t *datapos, const void *obj)
{
    const april_graph_factor_t *factor = obj;

    for (int i = 0; i < factor->nnodes; i++)
        encode_u32(data, datapos, factor->nodes[i]);

    for (int i = 0; i < factor->length; i++)
        encode_f64(data, datapos, factor->u.common.z[i]);

    if (factor->u.common.ztruth) {
        encode_u8(data, datapos, factor->length);

        for (int i = 0; i < factor->length; i++)
            encode_f64(data, datapos, factor->u.common.ztruth[i]);
    } else {
        encode_u8(data, datapos, 0);
    }

    for (int i = 0; i < factor->length; i++)
        encode_f64(data, datapos, factor->u.common.W->data[i]);

    april_graph_attr_t *attr = factor->attr;
    stype_encode_object(data, datapos, attr ? attr->stype : NULL, attr);
}

static void *april_graph_factor_b_decode(const stype_t *stype, const uint8_t *data, uint32_t *datapos, uint32_t datalen)
{
    int a = decode_u32(data, datapos, datalen);
    int b = decode_u32(data, datapos, datalen);

    double z[FACTOR_LEN];
    for (int i = 0; i < FACTOR_LEN; i++)
        z[i] = decode_f64(data, datapos, datalen);

    double *ztruth = NULL;
    if (decode_u8(data, datapos, datalen)) {
        ztruth = malloc(FACTOR_LEN * sizeof(double));
        for (int i = 0; i < FACTOR_LEN; i++)
            ztruth[i] = decode_f64(data, datapos, datalen);
    }

    matd_t *W = matd_create(FACTOR_LEN, FACTOR_LEN);
    for (int i = 0; i < FACTOR_LEN; i++)
        W->data[i] = decode_f64(data, datapos, datalen);

    april_graph_factor_t *factor = april_graph_factor_b_create(a, b, z, ztruth, W);
    april_graph_attr_destroy(factor->attr);
    factor->attr = stype_decode_object(data, datapos, datalen, NULL);

    free(ztruth);
    matd_destroy(W);
    return factor;
}

const stype_t stype_april_factor_b = { .name = "april_graph_factor_b",
                                       .encode = april_graph_factor_b_encode,
                                       .decode = april_graph_factor_b_decode,
                                       .copy = NULL };

static april_graph_factor_eval_t* b_factor_eval(april_graph_factor_t* factor,
                                                april_graph_t* graph,
                                                april_graph_factor_eval_t *eval)
{
    //get pointers to all of the nodes attached to this edge
    april_graph_node_t* na;
    zarray_get(graph->nodes, factor->nodes[0], &na);
    assert(na->type == APRIL_GRAPH_NODE_B_TYPE);
    april_graph_node_t* nb;
    zarray_get(graph->nodes, factor->nodes[1], &nb);
    assert(nb->type == APRIL_GRAPH_NODE_B_TYPE);

    //Allocate a new eval if we weren't given one.
    //The first time this function is called for a given factor, a new eval
    //will have to be allocated, but after that the old eval is reused to save
    //on alloc/free costs.
    if(eval == NULL)
    {
        eval = calloc(1, sizeof(april_graph_factor_eval_t));

        //a bias factor should connect 2 nodes
        eval->jacobians = calloc(factor->nnodes + 1, sizeof(matd_t*));    // NB: NULL-terminated
        eval->jacobians[0] = matd_create(factor->length, 3);  // old bias node  [len(factor or zhat) x len(node)]
        eval->jacobians[1] = matd_create(factor->length, 3);  // new bias node  [len(factor or zhat) x len(node)]

        eval->length = factor->length;
        eval->r = calloc(factor->length, sizeof(double));     //residual
        eval->W = matd_create(factor->length, factor->length);
    }

    //== Calculate zhat ==
    double zhat[] = { na->state[0] - nb->state[0] };

    //calculate the jacobians
    //partial derivatives of zhat WRT node 0 [na]
    matd_set_data(eval->jacobians[0], (double[]){
        1});
    //partial derivatives of zhat WRT node 0 [nb]
    matd_set_data(eval->jacobians[1], (double[]){
        -1});

    //Calculate the residual, which is the difference between zhat and the
    //factor's current z.  See pages [10, 11] of Ed's "multi-robot mapping"
    //"textbook" for more info.
    for (int i = 0; i < factor->length; i++) {
        eval->r[i] = factor->u.common.z[i] - zhat[i];
    }

    //copy over W from the factor to the eval.
    memcpy(eval->W->data, factor->u.common.W->data, factor->length *
           factor->length * sizeof(double));

    //calculate chi^2.  Consult page 11 of Ed's "multi-robot mapping"
    //"textbook".  Chi^2 = r'*W*r.  Use X = W * r
    double X[] = { MATD_EL(eval->W, 0, 0) * eval->r[0] };

    eval->chi2 = eval->r[0] * X[0];

    return eval;
}

april_graph_factor_t *april_graph_factor_b_create(int a, int b, const double
                                                   *z, const double *ztruth,
                                                   const matd_t *W)
{
    april_graph_factor_t *factor = calloc(1, sizeof(april_graph_factor_t));

    factor->stype = &stype_april_factor_b;
    factor->type = APRIL_GRAPH_FACTOR_B_TYPE;
    factor->nnodes = 2;
    factor->nodes = calloc(factor->nnodes, sizeof(int));
    factor->nodes[0] = a; // Older node
    factor->nodes[1] = b; // Newer node
    factor->length = 1;

    factor->copy = b_factor_copy;
    factor->eval = b_factor_eval;
    factor->destroy = b_factor_destroy;

    factor->u.common.z = doubles_dup(z, factor->length);
    factor->u.common.ztruth = doubles_dup(ztruth, factor->length);
    factor->u.common.W = matd_copy(W);

    return factor;
}

///////////////////////////////////////////////////////
//                     node impl                     //
///////////////////////////////////////////////////////
/**
 * n points to a node that should be modified and dstate points to doubles
 * which represent a change in state.
 */
static void b_node_update(april_graph_node_t* n, double* dstate)
{
    for(int i = 0; i < n->length; i++)
    {
        n->state[i] += dstate[i];
    }
}

//TODO: implement
static april_graph_node_t* b_node_copy(april_graph_node_t* node)
{
    assert(0);
    return NULL;
}

/**
 * Frees all of the resources that were allocated for a bias node.
 */
static void b_node_destroy(april_graph_node_t* node)
{
    free(node->state);
    free(node->init);
    free(node->truth);
    free(node);
}

/**
 * Serializes an april graph bias node (?)
 */
static void april_graph_node_b_encode(const stype_t* stype, uint8_t *data,
                                      uint32_t *datapos, const void *obj)
{
    const april_graph_node_t *node = obj;

    for (int i = 0; i < node->length; i++)
        encode_f64(data, datapos, node->state[i]);

    if (node->init) {
        encode_u8(data, datapos, node->length);
        for (int i = 0; i < node->length; i++)
            encode_f64(data, datapos, node->init[i]);
    } else {
        encode_u8(data, datapos, 0);
    }

    if (node->truth) {
        encode_u8(data, datapos, node->length);
        for (int i = 0; i < node->length; i++)
            encode_f64(data, datapos, node->truth[i]);
    } else {
        encode_u8(data, datapos, 0);
    }

    april_graph_attr_t *attr = node->attr;
    stype_encode_object(data, datapos, attr ? attr->stype : NULL, attr);
}

/**
 * Deserialises an april graph bias node from inside a bytestream given by a
 * uint8_t*
 */
static void* april_graph_node_b_decode(const stype_t *stype, const uint8_t *data,
                                        uint32_t *datapos, uint32_t datalen)
{
    double state[FACTOR_LEN];

    for (int i = 0; i < FACTOR_LEN; i++)
        state[i] = decode_f64(data, datapos, datalen);

    double *init = NULL;
    if (decode_u8(data, datapos, datalen)) {
        init = malloc(FACTOR_LEN * sizeof(double));
        for (int i = 0; i < FACTOR_LEN; i++)
            init[i] = decode_f64(data, datapos, datalen);
    }

    double *truth = NULL;
    if (decode_u8(data, datapos, datalen)) {
        truth = malloc(FACTOR_LEN * sizeof(double));
        for (int i = 0; i < FACTOR_LEN; i++)
            truth[i] = decode_f64(data, datapos, datalen);
    }

    april_graph_node_t *node = april_graph_node_b_create(state, init, truth);
    april_graph_attr_destroy(node->attr);
    node->attr = stype_decode_object(data, datapos, datalen, NULL);

    free(init);
    free(truth);
    return node;
}

const stype_t stype_april_node_b = { .name = "april_graph_node_b",
                                      .encode = april_graph_node_b_encode,
                                      .decode = april_graph_node_b_decode,
                                      .copy = NULL};

/**
 * Creates a new bias node, allocating all of the resources necessary.
 * State is [b] where,
 *    b: the additive offset bias, for example local device to global time.
 */
april_graph_node_t* april_graph_node_b_create(double* state, double* init,
                                               double* truth)
{
    april_graph_node_t* node = calloc(1, sizeof(april_graph_node_t));
    node->type = APRIL_GRAPH_NODE_B_TYPE;
    node->length = FACTOR_LEN;
    node->state = doubles_dup(state, node->length);
    node->init = doubles_dup(init, node->length);
    node->truth = doubles_dup(truth, node->length);

    node->update = b_node_update;
    node->copy = b_node_copy;
    node->destroy = b_node_destroy;

    node->stype = &stype_april_node_b;
    return node;
}

void april_graph_b_stype_init()
{
    stype_register(&stype_april_factor_b);
    stype_register(&stype_april_node_b);
}
