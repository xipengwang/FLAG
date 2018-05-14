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

#include "dijkstra_xyt.h"

/////////////////////////////////////////////////
#define TNAME dijkstra_xyt_edge_heap
#define TVALTYPE dijkstra_xyt_edge_t*
#include "common/tmaxheap_impl.h"
#undef TNAME
#undef TVALTYPE

/////////////////////////////////////////////////
#define TNAME dijkstra_xyt_edge_array
#define TVALTYPE dijkstra_xyt_edge_t*
#include "common/tarray_impl.h"
#undef TNAME
#undef TVALTYPE

#include "april_graph.h"
#include "common/doubles.h"

static void handle_factor(april_graph_t *graph, dijkstra_xyt_edge_array_t **arrays, april_graph_factor_t *factor)
{
    if (factor->type == APRIL_GRAPH_FACTOR_XYT_TYPE) {
        matd_t *P = matd_op("M^-1", factor->u.common.W);
        dijkstra_xyt_edge_t *a = calloc(1, sizeof(dijkstra_xyt_edge_t));
        dijkstra_xyt_edge_t *b = calloc(1, sizeof(dijkstra_xyt_edge_t));

        doubles_xytcov_copy(factor->u.common.z, P->data, a->xyt, a->cov);
        a->a = factor->nodes[0];
        a->b = factor->nodes[1];

        doubles_xytcov_inv(a->xyt, a->cov, b->xyt, b->cov);
        b->a = factor->nodes[1];
        b->b = factor->nodes[0];

        dijkstra_xyt_edge_array_add(arrays[a->a], &a);
        dijkstra_xyt_edge_array_add(arrays[b->a], &b);

        matd_destroy(P);

    } else if (factor->type == APRIL_GRAPH_FACTOR_MAX_TYPE) {

        int nfactors = factor->u.max.nfactors;

        int best_i = -1;
        double best_chi2 = DBL_MAX;
        april_graph_factor_eval_t *eval = NULL;

        for (int i = 0; i < nfactors; i++) {
            april_graph_factor_t *f = factor->u.max.factors[i];
            eval = f->eval(f, graph, eval);
            if (eval->chi2 < best_chi2) {
                best_chi2 = eval->chi2 + factor->u.max.logw[i];
                best_i = i;
            }
        }

        handle_factor(graph, arrays, factor->u.max.factors[best_i]);
    }
}

dijkstra_xyt_t *dijkstra_xyt_create(april_graph_t *graph, int refpos)
{
    int nposes = zarray_size(graph->nodes);

    // first, create an array of outgoing edges.
    dijkstra_xyt_edge_array_t **arrays = calloc(nposes, sizeof(dijkstra_xyt_edge_array_t*));
    for (int i = 0; i < nposes; i++)
        arrays[i] = dijkstra_xyt_edge_array_create();

    for (int i = 0; i < zarray_size(graph->factors); i++) {
        april_graph_factor_t *factor;
        zarray_get(graph->factors, i, &factor);

        handle_factor(graph, arrays, factor);
    }

    dijkstra_xyt_edge_heap_t *heap = dijkstra_xyt_edge_heap_create();
    dijkstra_xyt_edge_t **proj = calloc(nposes, sizeof(dijkstra_xyt_edge_t*));

    if (1) {
        dijkstra_xyt_edge_t *root_edge = calloc(1, sizeof(dijkstra_xyt_edge_t));
        root_edge->a = refpos;
        root_edge->b = refpos;
        dijkstra_xyt_edge_heap_add(heap, &root_edge, 0);
    }

    while (1) {
        // get the best heap from the edge
        dijkstra_xyt_edge_t *edge0;
        float score0;
        if (!dijkstra_xyt_edge_heap_remove_max(heap, &edge0, &score0))
            break;

        // if we've already found a path to this edge,
        if (proj[edge0->b] != NULL) {
            free(edge0);
            continue;
        }

        proj[edge0->b] = edge0;

        // expand edge0 by all of the outgoing neighbors of edge0.b
        dijkstra_xyt_edge_array_t *edges = arrays[edge0->b];

        for (int j = 0; j < dijkstra_xyt_edge_array_size(edges); j++) {
            dijkstra_xyt_edge_t *edge;
            dijkstra_xyt_edge_array_get(edges, j, &edge);

            // if we've already arrived at an edge, terminate search.
            if (proj[edge->b])
                continue;

            dijkstra_xyt_edge_t *newedge = calloc(1, sizeof(dijkstra_xyt_edge_t));

            doubles_xytcov_mul(edge0->xyt, edge0->cov, edge->xyt, edge->cov, newedge->xyt, newedge->cov);
            newedge->a = refpos;
            newedge->b = edge->b;

            // XXX do we want trace or determinant?
            float trace = newedge->cov[0] + newedge->cov[4] + newedge->cov[8];
            dijkstra_xyt_edge_heap_add(heap, &newedge, -trace);
        }
    }

    for (int i = 0; i < nposes; i++) {
        dijkstra_xyt_edge_array_t *edges = arrays[i];
        for (int j = 0; j < dijkstra_xyt_edge_array_size(edges); j++) {
            dijkstra_xyt_edge_t *edge;
            dijkstra_xyt_edge_array_get(edges, j, &edge);
            free(edge);
        }

        dijkstra_xyt_edge_array_destroy(edges);
    }
    free(arrays);

    dijkstra_xyt_edge_heap_destroy(heap);

    dijkstra_xyt_t *dijkstra = calloc(1, sizeof(dijkstra_xyt_t));
    dijkstra->nedges = nposes;
    dijkstra->edges = proj;
    dijkstra->refpos = refpos;

    return dijkstra;
}

void dijkstra_xyt_destroy(dijkstra_xyt_t *dijkstra)
{
    if (!dijkstra)
        return;

    for (int i = 0; i < dijkstra->nedges; i++)
        free(dijkstra->edges[i]);
    free(dijkstra->edges);
    free(dijkstra);
}
