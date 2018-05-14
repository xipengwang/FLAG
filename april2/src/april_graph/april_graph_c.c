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
 * This april graph factor is meant for working with time of flight radio
 * ranging measurements, hence the factor is named "c" for "clock."
 *
 * This factor will normally connect 4 nodes: one xyt pose node (for the robot),
 * one xy pose node (for the stationary beacon), and 2 bias nodes, one bias node
 * for the beacon on the robot and one bias node for the stationary beacon.
 *
 * Because the types of nodes that we link together with a clock factor are all
 * different, it's important that we establish some kind of order so that our
 * eval matricies are consistent with how they are positioned.  Let's define
 * the node order now to be:
 *     robot xyt, fixed beacon xy, bias, ....
 */

static april_graph_factor_t* c_factor_copy(april_graph_factor_t* factor)
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

    next->u.common.z = doubles_dup(factor->u.common.z, 1);
    next->u.common.ztruth = doubles_dup(factor->u.common.ztruth, 1);
    next->u.common.W = matd_copy(factor->u.common.W);

    return next;
}

static void c_factor_destroy(april_graph_factor_t* factor)
{
    free(factor->nodes);
    free(factor->u.common.z);
    free(factor->u.common.ztruth);
    matd_destroy(factor->u.common.W);
    free(factor);
}

static april_graph_factor_eval_t* c_factor_eval(april_graph_factor_t* factor,
                                                april_graph_t* graph,
                                                april_graph_factor_eval_t *ev)
{
    //allocate a new eval if we weren't given one
    if(ev == NULL)
    {
        ev = calloc(1, sizeof(april_graph_factor_eval_t));

        ev->jacobians = calloc(factor->length + 1, sizeof(matd_t*)); // NB: NULL-terminated

        // Jacobians are size [1 x len(node)]
        ev->jacobians[0] = matd_create(1, 3);        //xyt node pose
        ev->jacobians[1] = matd_create(1, 2);        //xy node pose
        ev->jacobians[2] = matd_create(1, 3);        //rx node bias
        ev->jacobians[3] = matd_create(1, 3);        //tx node bias

        //the residual and the W=Sigma^-1 both have dimension 1x1
        ev->length = 1;
        ev->r = calloc(ev->length, sizeof(double));          //residual
        ev->W = matd_create(ev->length, ev->length);
    }

    //Get pointers to all of the nodes attached to this edge.
    april_graph_node_t* ns[4];
    for(int i = 0; i < factor->nnodes; i++)
        zarray_get(graph->nodes, factor->nodes[i], &ns[i]);

    //calculate jacobians with respect to
    double xytrobt[3] = {ns[0]->state[0], ns[0]->state[1], ns[0]->state[2]};
    double xybeac[2]  = {ns[1]->state[0], ns[1]->state[1]};

    //calculate the distance squared and its derivative.
    double dx = xytrobt[0] - xybeac[0];
    double dy = xytrobt[1] - xybeac[1];
    double dsq = sq(dx) + sq(dy);
    double dder = 0.5 * pow(dsq, -0.5);
    const double lsb_per_meter = (1E12 / (300000000 * 15.65));

    //Calculate predicted observation.
    //The observation in a april_graph_c factor is the difference in send and
    //recieve time of a radio message between 2 beacons.  This time difference
    //observation is a function of the distance between the robot and
    //stationary beacon with the biases of the transmitting and recieving
    //radios put in "additively".  Namely:
    //
    // (Z denotes the observation, that is, the time difference between the
    //  transmission and reception of the message)
    //
    //Z = (distance / (speed of light)) + rx_bias - tx_bias

    // Units of zhat are in DW register LSBs; 1 bit is 15.65 picoseconds.
    double distance_in_time = (1E12 * (pow(dsq, 0.5) / 300000000.0)) * (1.0 / 15.65);
    double zhat = distance_in_time + ns[2]->state[0] - ns[3]->state[0];

    printf("[april_graph_c] z: %f\n", factor->u.common.z[0]);
    printf("[april_graph_c] bias_robot: %f, bias_anchor: %f\n", ns[2]->state[0], ns[3]->state[0]);
    printf("[april_graph_c] distance_in_time: %f, dsq: %f, zhat: %f\n", distance_in_time, dsq, zhat);
    printf("[april_graph_c] xytrobt[0] = %f, xybeac[0] = %f, dx = %f\n", xytrobt[0], xybeac[0], dx);
    printf("[april_graph_c] xytrobt[1] = %f, xybeac[1] = %f, dy = %f\n", xytrobt[1], xybeac[1], dy);

    zhat = fmod(zhat, (double)(((int64_t) 1) << 40));
    if(zhat < 0.0)
        zhat += (double)(((int64_t) 1) << 40);

    //partial derivatives of zhat WRT the robot pose node
    //  [ (robotx - beacx) / (speed of light * distance) ]
    //  [ (roboty - beacy) / (speed of light * distance) ]
    //  [               zero                             ]
    matd_set_data(ev->jacobians[0], (double[]){
                  (2.0 * dder * dx) * lsb_per_meter, (2.0 * dder * dy) * lsb_per_meter, 0});

    //partial derivatives of zhat WRT the beacon "pose" ndoe
    matd_set_data(ev->jacobians[1], (double[]){
                  (-2.0 * dder * dx) * lsb_per_meter, (-2.0 * dder * dy) * lsb_per_meter});

    //partial derivates WRT bais.
    //In this implementation of the system, the robot is always the receiver and
    //the stationary beacon is always the transmitter.
    matd_set_data(ev->jacobians[2], (double[]){1, 0, 0});    //robot
    matd_set_data(ev->jacobians[3], (double[]){-1, 0, 0});   //stationary beacon.

    //Calculate the residual.  This is given by the difference between the
    //measured z and the factor's current z.
    //Check pages [10, 11] of Ed's "multi-robot mapping" "textbook" for more
    //info.
    //
    //Note that R is 1-d.  It is just a single time representing the difference
    //between 2 clocks.
    ev->r[0] = factor->u.common.z[0] - zhat;

    //copy over W (information matrix) from the factor to the eval.
    memcpy(ev->W->data, factor->u.common.W->data, ev->length * ev->length * sizeof(double));

    //Calculate chi^2.  Consult page 11 of Ed's "multi-robot mapping"
    //"textbook".  Chi^2 = r'*W*r.
    //Use X = W * r
    double X[1] = {MATD_EL(ev->W, 0, 0) * ev->r[0]};
    ev->chi2 = ev->r[0] * X[0];

    return ev;
}

static void april_graph_factor_c_encode(const stype_t *stype, uint8_t *data,
                                          uint32_t *datapos, const void *obj)
{
    const april_graph_factor_t *factor = obj;

    for (int i = 0; i < factor->length; i++)
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

static void *april_graph_factor_c_decode(const stype_t *stype, const uint8_t *data, uint32_t *datapos, uint32_t datalen)
{
#define LEN 4 // factor->length
    int a = decode_u32(data, datapos, datalen);
    int b = decode_u32(data, datapos, datalen);
    int c = decode_u32(data, datapos, datalen);
    int d = decode_u32(data, datapos, datalen);

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

    april_graph_factor_t *factor = april_graph_factor_c_create(a, b, c, d, z, ztruth, W);
    april_graph_attr_destroy(factor->attr);
    factor->attr = stype_decode_object(data, datapos, datalen, NULL);

    free(ztruth);
    matd_destroy(W);
    return factor;
}

const stype_t stype_april_factor_c = { .name = "april_graph_factor_c",
                                       .encode = april_graph_factor_c_encode,
                                       .decode = april_graph_factor_c_decode,
                                       .copy = NULL };

// node a: xyt
// node b: xy
// node c: b   (robot)
// node d: b   (stationary beacon)
april_graph_factor_t *april_graph_factor_c_create(int a, int b, int c, int d,
                                                  double *z,
                                                  double *ztruth,
                                                  matd_t *W)
{
    april_graph_factor_t *factor = calloc(1, sizeof(april_graph_factor_t));

    int robot_idx = a;
    int beacon_idx = b;
    int robot_bias = c;
    int beacon_bias = d;
    factor->stype = &stype_april_factor_c;
    factor->type = APRIL_GRAPH_FACTOR_C_TYPE;
    factor->nnodes = 4;
    factor->nodes = calloc(factor->nnodes, sizeof(int));
    factor->nodes[0] = robot_idx;
    factor->nodes[1] = beacon_idx;
    factor->nodes[2] = robot_bias;
    factor->nodes[3] = beacon_bias;
    factor->length = 1; // Length of z and zhat

    factor->copy = c_factor_copy;
    factor->eval = c_factor_eval;
    factor->destroy = c_factor_destroy;

    factor->u.common.z = doubles_dup(z, factor->length);
    factor->u.common.ztruth = doubles_dup(ztruth, factor->length);
    factor->u.common.W = matd_copy(W);

    return factor;
}
