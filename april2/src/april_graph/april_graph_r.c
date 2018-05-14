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
// R Factor
static april_graph_factor_t* r_factor_copy(april_graph_factor_t *factor)
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

    next->u.common.z = doubles_dup(factor->u.common.z, 1);
    next->u.common.ztruth = doubles_dup(factor->u.common.ztruth, 1);
    next->u.common.W = matd_copy(factor->u.common.W);

    if(factor->type == APRIL_GRAPH_FACTOR_R_OFFSET_TYPE) {
        double * offset = malloc(3*sizeof(double));
        offset[0] = ((double*) factor->u.common.impl)[0];
        offset[1] = ((double*) factor->u.common.impl)[1];
        offset[2] = ((double*) factor->u.common.impl)[2];
    }

    return next;
}

static april_graph_factor_eval_t* r_factor_eval(april_graph_factor_t *factor, april_graph_t *graph, april_graph_factor_eval_t *eval)
{
    assert(factor->type == APRIL_GRAPH_FACTOR_R_TYPE
           || (factor->type == APRIL_GRAPH_FACTOR_R_OFFSET_TYPE && factor->u.common.impl != NULL));

    april_graph_node_t *na, *nb;
    zarray_get(graph->nodes, factor->nodes[0], &na);
    assert(na->type == APRIL_GRAPH_NODE_XYT_TYPE ||
           na->type == APRIL_GRAPH_NODE_XYZ_TYPE ||
           na->type == APRIL_GRAPH_NODE_XY_TYPE);
    zarray_get(graph->nodes, factor->nodes[1], &nb);
    assert(nb->type == APRIL_GRAPH_NODE_XYT_TYPE ||
           nb->type == APRIL_GRAPH_NODE_XYZ_TYPE ||
           nb->type == APRIL_GRAPH_NODE_XY_TYPE);
    if (eval == NULL) {
        eval = calloc(1, sizeof(april_graph_factor_eval_t));
        eval->jacobians = calloc(3, sizeof(matd_t*)); // NB: NULL-terminated
        eval->jacobians[0] = matd_create(1,na->length);
        eval->jacobians[1] = matd_create(1,nb->length);
        eval->r = calloc(1, sizeof(double));
        eval->W = matd_create(1,1);
    }

    // z = ((xa - xb)^2 + (ya-yb)^2)^.5
    // where (xa,ya) is the position of the robot, and (xb,yb) is the position of the beacon
    //
    // partial derivatives of zhat WRT node 0 [xa ya ta]

    double xa = na->state[0], ya = na->state[1];
    double xb = nb->state[0], yb = nb->state[1];

    double theta_j = 0;

    // Translational offset if factor is offset type and base node is xyt type
    if(factor->type == APRIL_GRAPH_FACTOR_R_OFFSET_TYPE
       && na->type == APRIL_GRAPH_NODE_XYT_TYPE){
        double xyt[3];
        double *offset = factor->u.common.impl;
        doubles_xyt_mul(na->state, offset, xyt);
        xa = xyt[0];
        ya = xyt[1];

        double ta = na->state[2];
        theta_j = 2*(xa-xb)*(-offset[0]*sin(ta) - offset[1]*cos(ta)) + 2*(ya-yb)*(-offset[0]*cos(ta) - offset[1]*sin(ta));
    }

    // Height offset
    if(nb->type == APRIL_GRAPH_NODE_XYZ_TYPE) {
        double za = 0;
        if(factor->type == APRIL_GRAPH_FACTOR_R_OFFSET_TYPE)
            za = ((double*) factor->u.common.impl)[2];

        double zb = nb->state[2];
        double d = sq(xa - xb) + sq(ya - yb) + sq(za - zb);
        double dder = 0.5 * pow(d, -.5);
        // partial derivatives of zhat WRT node 0 [xa ya 0]
        matd_set_data(eval->jacobians[0], (double[]) {
                dder * 2 * (xa - xb),     dder * 2 * (ya - yb), dder*theta_j });

        // partial derivatives of zhat WRT node 1 [xb yb zb]
        matd_set_data(eval->jacobians[1], (double[]) {
            -dder * 2 * (xa - xb) ,   - dder * 2 * (ya - yb), -dder * 2 *(za - zb) });

        eval->r[0] = factor->u.common.z[0] - sqrt(d);
        printf("za %f zb %f r %f jbz %f\n", za, zb, eval->r[0], -dder * 2 *(za - zb));
    } else if (na->type == nb->type && na->type == APRIL_GRAPH_NODE_XY_TYPE) {
        double d = sq(xa - xb) + sq(ya - yb);
        double dder = 0.5 * pow(d, -.5);

        // partial derivatives of zhat WRT node 0 [xa ya]
        matd_set_data(eval->jacobians[0], (double[]) {
            dder * 2 * (xa - xb),     dder * 2 * (ya - yb) });

        // partial derivatives of zhat WRT node 1 [xb yb]
        matd_set_data(eval->jacobians[1], (double[]) {
            -dder * 2 * (xa - xb) ,   - dder * 2 * (ya - yb) });

        eval->r[0] = factor->u.common.z[0] - sqrt(d);
    } else {
        double d = sq(xa - xb) + sq(ya - yb);
        double dder = 0.5 * pow(d, -.5);

        // partial derivatives of zhat WRT node 0 [xa ya ta]
        matd_set_data(eval->jacobians[0], (double[]) {
            dder * 2 * (xa - xb),     dder * 2 * (ya - yb), dder*theta_j });

        // partial derivatives of zhat WRT node 1 [xb yb tb]
        matd_set_data(eval->jacobians[1], (double[]) {
            -dder * 2 * (xa - xb) ,   - dder * 2 * (ya - yb), 0 });

        eval->r[0] = factor->u.common.z[0] - sqrt(d);
    }

    assert(!isnan(eval->r[0]));

    memcpy(eval->W->data, factor->u.common.W->data, 1*1*sizeof(double));

    // chi^2 = r'*W*r, via X = W*r
    eval->chi2 = eval->r[0] * eval->r[0] * factor->u.common.W->data[0];

    return eval;
}

static void r_factor_destroy(april_graph_factor_t *factor)
{
    free(factor->nodes);
    free(factor->u.common.z);
    free(factor->u.common.ztruth);
    matd_destroy(factor->u.common.W);
    free(factor->u.common.impl);
    free(factor);
}

static void april_graph_factor_r_encode(const stype_t *stype, uint8_t *data, uint32_t *datapos, const void *obj)
{
    const april_graph_factor_t *factor = obj;

    encode_u32(data, datapos, factor->nodes[0]);
    encode_u32(data, datapos, factor->nodes[1]);

    for (int i = 0; i < 1; i++)
        encode_f64(data, datapos, factor->u.common.z[i]);

    if (factor->u.common.ztruth) {
        encode_u8(data, datapos, 1);

        for (int i = 0; i < 1; i++)
            encode_f64(data, datapos, factor->u.common.ztruth[i]);
    } else {
        encode_u8(data, datapos, 0);
    }

    for (int i = 0; i < 1; i++)
        encode_f64(data, datapos, factor->u.common.W->data[i]);

    april_graph_attr_t *attr = factor->attr;
    stype_encode_object(data, datapos, attr ? attr->stype : NULL, attr);

    if(factor->type == APRIL_GRAPH_FACTOR_R_OFFSET_TYPE) {
        encode_u8(data, datapos, 1);
        for (int i = 0; i < 3; i++)
            encode_f64(data, datapos, ((double*) factor->u.common.impl)[i]);
    } else {
        encode_u8(data, datapos, 0);
    }
}

static void *april_graph_factor_r_decode(const stype_t *stype, const uint8_t *data, uint32_t *datapos, uint32_t datalen)
{
    int a = decode_u32(data, datapos, datalen);
    int b = decode_u32(data, datapos, datalen);

    double z[3];
    for (int i = 0; i < 1; i++)
        z[i] = decode_f64(data, datapos, datalen);

    double *ztruth = NULL;
    if (decode_u8(data, datapos, datalen)) {
        ztruth = malloc(1*sizeof(double));
        for (int i = 0; i < 1; i++)
            ztruth[i] = decode_f64(data, datapos, datalen);
    }

    matd_t *W = matd_create(1, 1);
    for (int i = 0; i < 1; i++)
        W->data[i] = decode_f64(data, datapos, datalen);

    april_graph_factor_t *factor = april_graph_factor_r_create(a, b, z, ztruth, W);
    april_graph_attr_destroy(factor->attr);
    factor->attr = stype_decode_object(data, datapos, datalen, NULL);

    if (decode_u8(data, datapos, datalen)) {
        factor->type = APRIL_GRAPH_FACTOR_R_OFFSET_TYPE;
        double * offset = malloc(3*sizeof(double));
        for (int i = 0; i < 3; i++)
            offset[i] = decode_f64(data, datapos, datalen);
        factor->u.common.impl = offset;
    }

    free(ztruth);
    matd_destroy(W);
    return factor;
}

const stype_t stype_april_factor_r = { .name = "april_graph_factor_r",
                                       .encode = april_graph_factor_r_encode,
                                       .decode = april_graph_factor_r_decode,
                                       .copy = NULL };

// node a: xyt
// node b: xy
april_graph_factor_t *april_graph_factor_r_create(int a, int b, double *z, double *ztruth, matd_t *W)
{
    april_graph_factor_t *factor = calloc(1, sizeof(april_graph_factor_t));

    factor->stype = &stype_april_factor_r;
    factor->type = APRIL_GRAPH_FACTOR_R_TYPE;
    factor->nnodes = 2;
    factor->nodes = calloc(2, sizeof(int));
    factor->nodes[0] = a;
    factor->nodes[1] = b;
    factor->length = 1;

    factor->copy = r_factor_copy;
    factor->eval = r_factor_eval;
    factor->destroy = r_factor_destroy;

    factor->u.common.z = doubles_dup(z, 1);
    factor->u.common.ztruth = doubles_dup(ztruth, 1);
    factor->u.common.W = matd_copy(W);

    return factor;
}

april_graph_factor_t *april_graph_factor_r_offset_create(int a, int b, double *z, double *ztruth, matd_t *W, double *xyz_offset)
{
    april_graph_factor_t *factor = calloc(1, sizeof(april_graph_factor_t));

    factor->stype = &stype_april_factor_r;
    factor->type = APRIL_GRAPH_FACTOR_R_OFFSET_TYPE;
    factor->nnodes = 2;
    factor->nodes = calloc(2, sizeof(int));
    factor->nodes[0] = a;
    factor->nodes[1] = b;
    factor->length = 1;

    factor->copy = r_factor_copy;
    factor->eval = r_factor_eval;
    factor->destroy = r_factor_destroy;

    factor->u.common.z = doubles_dup(z, 1);
    factor->u.common.ztruth = doubles_dup(ztruth, 1);
    factor->u.common.W = matd_copy(W);

    factor->u.common.impl = doubles_dup(xyz_offset, 3);

    return factor;
}

/////////////////////////////////////////////////////////////////////////////////////////

void april_graph_r_stype_init()
{
    stype_register(&stype_april_factor_r);
}
