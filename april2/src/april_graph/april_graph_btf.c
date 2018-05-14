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

/**
 * This file contains an implementation for an aprilgraph node and edge that
 * represent a radio bias.
 */

///////////////////////////////////////////////////////
//                    factor impl                    //
///////////////////////////////////////////////////////

static april_graph_factor_t* btf_factor_copy(april_graph_factor_t* factor)
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

static void btf_factor_destroy(april_graph_factor_t* factor)
{
    free(factor->nodes);
    free(factor->u.common.z);
    free(factor->u.common.ztruth);
    matd_destroy(factor->u.common.W);
    free(factor);
}

static void april_graph_factor_btf_encode(const stype_t *stype, uint8_t *data,
                                          uint32_t *datapos, const void *obj)
{
    const april_graph_factor_t *factor = obj;

    encode_u32(data, datapos, factor->nodes[0]);
    encode_u32(data, datapos, factor->nodes[1]);

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

static void *april_graph_factor_btf_decode(const stype_t *stype, const uint8_t *data, uint32_t *datapos, uint32_t datalen)
{
#define LEN 3 // factor->length
    int a = decode_u32(data, datapos, datalen);
    int b = decode_u32(data, datapos, datalen);

    double z[LEN];
    for (int i = 0; i < LEN; i++)
        z[i] = decode_f64(data, datapos, datalen);

    double *ztruth = NULL;
    if (decode_u8(data, datapos, datalen)) {
        ztruth = malloc(LEN * sizeof(double));
        for (int i = 0; i < LEN; i++)
            ztruth[i] = decode_f64(data, datapos, datalen);
    }

    matd_t *W = matd_create(LEN, LEN);
    for (int i = 0; i < LEN; i++)
        W->data[i] = decode_f64(data, datapos, datalen);

    april_graph_factor_t *factor = april_graph_factor_btf_create(a, b, z, ztruth, W);
    april_graph_attr_destroy(factor->attr);
    factor->attr = stype_decode_object(data, datapos, datalen, NULL);

    free(ztruth);
    matd_destroy(W);
    return factor;
}

const stype_t stype_april_factor_btf = { .name = "april_graph_factor_btf",
                                        .encode = april_graph_factor_btf_encode,
                                        .decode = april_graph_factor_btf_decode,
                                        .copy = NULL };

static april_graph_factor_eval_t* btf_factor_eval(april_graph_factor_t* factor,
                                                  april_graph_t* graph,
                                                  april_graph_factor_eval_t *eval)
{
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
        eval->r = calloc(eval->length, sizeof(double));     //residual
        eval->W = matd_create(eval->length, factor->length);
    }

    //get pointers to all of the nodes attached to this edge
    april_graph_node_t* na;
    zarray_get(graph->nodes, factor->nodes[0], &na);
    april_graph_node_t* nb;
    zarray_get(graph->nodes, factor->nodes[1], &nb);

    //== Calculate zhat ==
    //Our best bet for the change in bias will be given by the bias of node a
    //with some constant drift added to it.  The constant drift increases as
    //time increases.
    // TODO: Only using dd for covariance, right?
    uint64_t* da_ptr = (uint64_t*)april_graph_node_attr_get(na, "txrx_time");
    uint64_t* db_ptr = (uint64_t*)april_graph_node_attr_get(nb, "txrx_time");
    assert(da_ptr);
    assert(db_ptr);
    uint64_t da = *da_ptr;
    uint64_t db = *db_ptr;
    double dd = 0;
    if (db > da) // Normal calculation
    {
        dd = (double)(db - da);
    }
    else         // Wraparound fix for DW 40 bit registers
    {
        dd = (double)((db + (((int64_t)1) << 40)) - da);
    }
    printf("[april_graph_btf] dd: %f\n", dd);

    double ba = na->state[0], ta = na->state[1], fa = na->state[2];
    double bb = nb->state[0], tb = nb->state[1], fb = nb->state[2];

    //We are going to assume that the bias is unchanged.  Not really ideal.
    //TODO: Any kind of wraparound handling? I don't think we need any here, just on device time
    double zhat[3] = {(tb - ta) + (bb - ba),
                      (tb - ta) * fa,
                       fb - fa};
    printf("[april_graph_btf] zhat: \n\t%f\n\t%f\n\t%f\n", zhat[0], zhat[1], zhat[2]);

    //calculate the jacobians
    matd_set_data(eval->jacobians[0], (double[]){
        -1,     -1,     0,
        0,      -fa,    tb - ta,
        0,      0,      -1});
    matd_set_data(eval->jacobians[1], (double[]){
        1,      1,      0,
        0,      fa,     0,
        0,      0,      1});

    //Calculate the residual, which is the difference between zhat and the
    //factor's current z.  See pages [10, 11] of Ed's "multi-robot mapping"
    //"textbook" for more info.
    // TODO: Update residual
    for (int i = 0; i < eval->length; i++)
    {
        eval->r[i] = factor->u.common.z[i] - zhat[i];
    }

    //copy over W from the factor to the eval.
    memcpy(eval->W->data, factor->u.common.W->data, factor->length *
           factor->length * sizeof(double));

    //calculate chi^2.  Consult page 11 of Ed's "multi-robot mapping"
    //"textbook".  Chi^2 = r'*W*r.  Use X = W * r
    // TODO: Can't we just use a matrix multiply function call?
    double X[3] = { MATD_EL(eval->W, 0, 0) * eval->r[0] +
                    MATD_EL(eval->W, 0, 1) * eval->r[1] +
                    MATD_EL(eval->W, 0, 2) * eval->r[2],
                    MATD_EL(eval->W, 1, 0) * eval->r[0] +
                    MATD_EL(eval->W, 1, 1) * eval->r[1] +
                    MATD_EL(eval->W, 1, 2) * eval->r[2],
                    MATD_EL(eval->W, 2, 0) * eval->r[0] +
                    MATD_EL(eval->W, 2, 1) * eval->r[1] +
                    MATD_EL(eval->W, 2, 2) * eval->r[2] };

    eval->chi2 = eval->r[0] * X[0] + eval->r[1] * X[1] + eval->r[2] * X[2];

    return eval;
}

april_graph_factor_t *april_graph_factor_btf_create(int a, int b, const double
                                                   *z, const double *ztruth,
                                                   const matd_t *W)
{
    april_graph_factor_t *factor = calloc(1, sizeof(april_graph_factor_t));

    factor->stype = &stype_april_factor_btf;
    factor->type = APRIL_GRAPH_FACTOR_BTF_TYPE;
    factor->nnodes = 2;
    factor->nodes = calloc(factor->nnodes, sizeof(int));
    factor->nodes[0] = a; // Older node
    factor->nodes[1] = b; // Newer node
    factor->length = 3;   // TODO: This is number of measurements, right? If
                          // so, why not node->length? Doesn't make sense, because this should be blind of
                          // the node type. But then that leaves length == nnodes, but then why have length?

    factor->copy = btf_factor_copy;
    factor->eval = btf_factor_eval;
    factor->destroy = btf_factor_destroy;

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
static void btf_node_update(april_graph_node_t* n, double* dstate)
{
    for(int i = 0; i < n->length; i++)
    {
        n->state[i] += dstate[i];
    }
}

//TODO: implement
static april_graph_node_t* btf_node_copy(april_graph_node_t* node)
{
    assert(0);
    return NULL;
}

/**
 * Frees all of the resources that were allocated for a bias node.
 */
static void btf_node_destroy(april_graph_node_t* node)
{
    free(node->state);
    free(node->init);
    free(node->truth);
    free(node);
}

/**
 * Serializes an april graph bias node (?)
 */
static void april_graph_node_btf_encode(const stype_t* stype, uint8_t *data,
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
 * uint8_t* (?)
 */
static void* april_graph_node_btf_decode(const stype_t *stype, const uint8_t *data,
                                        uint32_t *datapos, uint32_t datalen)
{
#define LEN 3 // node->length

    double state[LEN];

    for (int i = 0; i < LEN; i++)
        state[i] = decode_f64(data, datapos, datalen);

    double *init = NULL;
    if (decode_u8(data, datapos, datalen)) {
        init = malloc(LEN * sizeof(double));
        for (int i = 0; i < LEN; i++)
            init[i] = decode_f64(data, datapos, datalen);
    }

    double *truth = NULL;
    if (decode_u8(data, datapos, datalen)) {
        truth = malloc(LEN * sizeof(double));
        for (int i = 0; i < LEN; i++)
            truth[i] = decode_f64(data, datapos, datalen);
    }

    april_graph_node_t *node = april_graph_node_btf_create(state, init, truth);
    april_graph_attr_destroy(node->attr);
    node->attr = stype_decode_object(data, datapos, datalen, NULL);

    free(init);
    free(truth);
    return node;
}

const stype_t stype_april_node_btf = { .name = "april_graph_node_btf",
                                      .encode = april_graph_node_btf_encode,
                                      .decode = april_graph_node_btf_decode,
                                      .copy = NULL};

/**
 * Creates a new bias node, allocating all of the resources necessary.
 * State is [b, t, f] where,
 *    b: the offset of local device to global time, or local_time = t + b.
 *    t: the node's estimate of the global time at the moment of tx or rx.
 *    f: the ratio of local clock time change (s) per global clock time change (s),
 *       such that delta_local = delta_global * f
 */
april_graph_node_t* april_graph_node_btf_create(double* state, double* init,
                                               double* truth)
{
    april_graph_node_t* node = calloc(1, sizeof(april_graph_node_t));
    node->type = APRIL_GRAPH_NODE_BTF_TYPE;
    node->length = 3;
    node->state = doubles_dup(state, node->length);
    node->init = doubles_dup(init, node->length);
    node->truth = doubles_dup(truth, node->length);

    node->update = btf_node_update;
    node->copy = btf_node_copy;
    node->destroy = btf_node_destroy;

    node->stype = &stype_april_node_btf;
    return node;
}

void april_graph_btf_stype_init()
{
    stype_register(&stype_april_factor_btf);
    stype_register(&stype_april_node_btf);
}
