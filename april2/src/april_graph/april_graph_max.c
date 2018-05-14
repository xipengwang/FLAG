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
#include <float.h>

/////////////////////////////////////////////////////////////////////////////////////////
// Multi Factor
static void april_graph_factor_max_encode(const stype_t *stype, uint8_t *data, uint32_t *datapos, const void *obj)
{
    const april_graph_factor_t *factor = obj;

    encode_u32(data, datapos, factor->u.max.nfactors);

    for (int i = 0; i < factor->u.max.nfactors; i++) {
        encode_f64(data, datapos, factor->u.max.logw[i]);

        april_graph_factor_t *f = factor->u.max.factors[i];
        stype_encode_object(data, datapos, f->stype, f);
    }
}

static void *april_graph_factor_max_decode(const stype_t *stype, const uint8_t *data, uint32_t *datapos, uint32_t datalen)
{
    int nfactors = decode_u32(data, datapos, datalen);
    assert(nfactors > 0);

    double logw[nfactors];
    april_graph_factor_t *factors[nfactors];

    for (int i = 0; i < nfactors; i++) {
        logw[i] = decode_f64(data, datapos, datalen);
        factors[i] = stype_decode_object(data, datapos, datalen, NULL);
    }

    return april_graph_factor_max_create(factors, logw, nfactors);
}

static const stype_t stype_april_factor_max = { .name = "april_graph_factor_max",
                                                .encode = april_graph_factor_max_encode,
                                                .decode = april_graph_factor_max_decode,
                                                .copy = NULL };

static void max_factor_destroy(april_graph_factor_t *factor)
{
    int nfactors = factor->u.max.nfactors;
    for (int i = 0; i < nfactors; i++) {
        april_graph_factor_t *f = factor->u.max.factors[i];
        f->destroy(f);
    }

    free(factor->u.max.logw);
    free(factor->u.max.factors);
    free(factor);
}


/* introduce a factor whose probability is the maximum of a number of "standard" factors:

   p(z|x) = max_i p(z_i | x_i, factor=i) p(factor=i)

   (This is equivalent to the standard max-mixtures formulation, where
   we've interpreted the weight w as p(factor=i).)

   When optimizing, the first step is to evaluate the term that has
   the maximum probability at the current state estimate.

*/

// p(z|x) = w*gamma*exp(chi2)
// log(p(z|x)) = log w + log gamma + chi2
static april_graph_factor_eval_t* max_factor_eval(april_graph_factor_t *factor, april_graph_t *graph, april_graph_factor_eval_t *eval)
{
    int nfactors = factor->u.max.nfactors;

    int best_i = -1;
    double best_chi2 = DBL_MAX;

    for (int i = 0; i < nfactors; i++) {
        april_graph_factor_t *f = factor->u.max.factors[i];
        eval = f->eval(f, graph, eval);
        eval->chi2 += factor->u.max.logw[i];
        if (eval->chi2 < best_chi2) {
            best_chi2 = eval->chi2;
            best_i = i;
        }
    }

    // a bit inefficient to re-evaluate the winner
    if (best_i != nfactors-1) {
        april_graph_factor_t *f = factor->u.max.factors[best_i];
        eval = f->eval(f, graph, eval);
        eval->chi2 += factor->u.max.logw[best_i];
    }

    return eval;
}

april_graph_factor_t* april_graph_factor_max_best(april_graph_factor_t *factor, april_graph_t *graph)
{
    int nfactors = factor->u.max.nfactors;

    int best_i = -1;
    double best_chi2 = DBL_MAX;
    april_graph_factor_eval_t* eval = NULL;

    for (int i = 0; i < nfactors; i++) {
        april_graph_factor_t *f = factor->u.max.factors[i];
        eval = f->eval(f, graph, eval);
        eval->chi2 += factor->u.max.logw[i];
        if (eval->chi2 < best_chi2) {
            best_chi2 = eval->chi2;
            best_i = i;
        }
    }

    april_graph_factor_eval_destroy(eval);

    assert(best_i != -1);
    return factor->u.max.factors[best_i];
}

// we take ownership of the factors, but not the array containing
// containing them. They will be freed when this factor is freed.
april_graph_factor_t *april_graph_factor_max_create(april_graph_factor_t **factors, double *logw, int nfactors)
{
    april_graph_factor_t *factor = calloc(1, sizeof(april_graph_factor_t));

    for (int i = 0; i < nfactors; i++) {
        assert(factors[i]->type == factors[0]->type);
        assert(factors[i]->nnodes == factors[0]->nnodes);
        assert(factors[i]->length == factors[0]->length);
        for (int j = 0; j < factors[i]->nnodes; j++)
            assert(factors[i]->nodes[j] == factors[0]->nodes[j]);
    }

    factor->type = APRIL_GRAPH_FACTOR_MAX_TYPE;
    factor->nnodes = factors[0]->nnodes;
    factor->nodes = calloc(factor->nnodes, sizeof(int));
    memcpy(factor->nodes, factors[0]->nodes, factor->nnodes * sizeof(int));
    factor->length = factors[0]->length;

    factor->copy = NULL; // XXX unimplemented max_factor_copy;
    factor->eval = max_factor_eval;
    factor->destroy = max_factor_destroy;

    factor->u.max.nfactors = nfactors;
    factor->u.max.logw = calloc(nfactors, sizeof(double));
    factor->u.max.factors = calloc(nfactors, sizeof(april_graph_factor_t*));
    for (int i = 0; i < nfactors; i++) {
        factor->u.max.factors[i] = factors[i];
        factor->u.max.logw[i] = logw[i];
    }

    factor->stype = &stype_april_factor_max;
    return factor;
}

void april_graph_max_stype_init()
{
    stype_register(&stype_april_factor_max);
}
