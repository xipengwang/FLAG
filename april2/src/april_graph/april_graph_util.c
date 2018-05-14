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

#include "april_graph_util.h"
#include "common/unionfind.h"


zarray_t * april_graph_get_connected_components(april_graph_t * graph)
{
    // Create an extra node for those edges which implicitly connect to
    // the coordinate frame

    int common_node = zarray_size(graph->nodes);
    unionfind_t *uf = unionfind_create(common_node + 1);

    for (int k = 0, sz = zarray_size(graph->factors); k< sz; k++) {
        april_graph_factor_t * factor = NULL;
        zarray_get(graph->factors, k, &factor);

        if (factor->nnodes == 1) {
            unionfind_connect(uf, factor->nodes[0], common_node);
        }

        for (int i = 0; i < factor->nnodes; i++)
            for (int j = i+1; j < factor->nnodes; j++)
                unionfind_connect(uf, factor->nodes[i], factor->nodes[j]);
    }

    // convert node index in orig graph to node index in sub graph
    int mapping[zarray_size(graph->nodes)];
    for (int i = 0, sz = zarray_size(graph->nodes); i <sz; i++)
        mapping[i] = -1;

    zhash_t * graphs = zhash_create(sizeof(int32_t), sizeof(april_graph_t *),
                                    zhash_uint32_hash, zhash_uint32_equals);
    // Copy the nodes BY REFERENCE, keeping track of their old and new idxs
    for (int gidx = 0, sz = zarray_size(graph->nodes); gidx < sz; gidx++) {
        int rep = unionfind_get_representative(uf, gidx);
        april_graph_node_t * node = NULL;
        zarray_get(graph->nodes, gidx, &node);

        april_graph_t * g = NULL;
        zhash_get(graphs, &rep, &g);
        if (g == NULL) {
            g = april_graph_create();
            zhash_put(graphs, &rep, &g, NULL, NULL);
        }

        mapping[gidx] = zarray_size(g->nodes);
        zarray_add(g->nodes, &node);
    }

    // Copy the factors BY VALUE, modify them, add to correct graph
    for (int eidx = 0, sz = zarray_size(graph->factors); eidx < sz; eidx++) {
        april_graph_factor_t * orig_factor = NULL;
        zarray_get(graph->factors, eidx, &orig_factor);

        int rep = unionfind_get_representative(uf, orig_factor->nodes[0]);

        for (int i = 1; i < orig_factor->nnodes; i++)
            assert(rep == unionfind_get_representative(uf, orig_factor->nodes[i]));

        april_graph_factor_t * new_factor = orig_factor->copy(orig_factor);
        assert(new_factor);

        for (int i = 0; i < new_factor->nnodes; i++)
            new_factor->nodes[i] = mapping[new_factor->nodes[i]];

        april_graph_t * g = NULL;
        zhash_get(graphs, &rep, &g);
        assert(g);

        zarray_add(g->factors, &new_factor);
    }

    unionfind_destroy(uf);

    zarray_t * graph_array = zhash_values(graphs);
    zhash_destroy(graphs);
    return graph_array;
}

static void factor_destroy(april_graph_factor_t * data)
{
    data->destroy(data);
}

static void special_graph_destroy(april_graph_t * g)
{
    zarray_vmap(g->factors, factor_destroy);

    zarray_destroy(g->factors);
    zarray_destroy(g->nodes);
    free(g);
}

void april_graph_connected_components_destroy(zarray_t * graphs)
{
    zarray_vmap(graphs, special_graph_destroy);
    zarray_destroy(graphs);
}
