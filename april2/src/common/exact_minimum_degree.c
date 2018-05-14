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

#include "zset.h"
#include "smatd.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <assert.h>

struct node
{
    int *neighbors;
    int nneighbors;
    int alloc;
};

int *exact_minimum_degree_ordering(smatd_t *mat)
{
    int nnodes = mat->nrows;
    struct node *nodes = calloc(nnodes, sizeof(struct node));

    for (int rowi = 0; rowi < mat->nrows; rowi++) {
        svecd_t *vec = &mat->rows[rowi];

        nodes[rowi].alloc = vec->nz * 2 + 8;
        nodes[rowi].neighbors = malloc(nodes[rowi].alloc * sizeof(int));
        for (int i = 0; i < vec->nz; i++) {
            if (vec->indices[i] == rowi)
                continue;

            nodes[rowi].neighbors[nodes[rowi].nneighbors++] = vec->indices[i];
        }
    }

    // live_nodes contains the indices of all the nodes that have not yet
    // been marginalized out. This array is used so that we can quickly
    // iterate over only the live nodes.
    int *live_nodes = malloc(nnodes * sizeof(int));
    int nlive_nodes = nnodes;

    // initially, all nodes are live.
    for (int i = 0; i < nnodes; i++)
        live_nodes[i] = i;

    // our output vector...
    int *ordering = calloc(nnodes, sizeof(int));

    // a hash set used to detect duplicate nodes when marginalizing
    // out nodes. See below.
    int *set = calloc(nnodes, sizeof(int));
    int settoken = 0;

    for (int orderingi = 0; orderingi < mat->nrows; orderingi++) {

        int bestnodei = live_nodes[0];
        int bestnneighbors = nodes[bestnodei].nneighbors;
        int best_live_node_idx = 0;

        // find the node with the fewest neighbors.
        for (int i = 1; i < nlive_nodes; i++) {
            int nodei = live_nodes[i];

            if (nodes[nodei].nneighbors < bestnneighbors) {
                bestnneighbors = nodes[nodei].nneighbors;
                bestnodei = nodei;
                best_live_node_idx = i;
            }
        }

        // marginalize-out bestnodei: every neighbor of bestnodei
        // becomes connected.
        for (int aidx = 0; aidx < nodes[bestnodei].nneighbors; aidx++) {
            int naidx = nodes[bestnodei].neighbors[aidx];

            struct node *na = &nodes[naidx];

            // na is the "destination" to which we will add all of the
            // neighbors of bestnodei.

            // make the 'set' contain the current neighbors.
            // set membership is denoted by set[i] == settoken.
            // we get "free" set tables by just incrementing settoken.
            settoken++;

            for (int i = 0; i < na->nneighbors; i++) {

                // while we're iterating, delete the bestnodei neighbor
                if (na->neighbors[i] == bestnodei) {
                    // shuffle delete
                    na->neighbors[i] = na->neighbors[na->nneighbors - 1];
                    na->nneighbors--;
                    i--;
                    continue;
                }

                set[na->neighbors[i]] = settoken;
            }

            // setting these set elements at this point will prevent us
            // from adding these nodes below.
            set[bestnodei] = settoken;
            set[naidx] = settoken;

            // now loop over the neighbors to add.
            for (int bidx = 0; bidx < nodes[bestnodei].nneighbors; bidx++) {

                int nbidx = nodes[bestnodei].neighbors[bidx];

                // set already contains this member
                if (set[nbidx] == settoken)
                    continue;

                // need to add nj to neighbors of ni.
                if (na->nneighbors + 1 >= na->alloc) {
                    // need to grow the neighbors array.
                    na->alloc *= 2;
                    assert(na->neighbors > 0 && na->alloc > 0);
                    na->neighbors = realloc(na->neighbors, na->alloc*sizeof(int));
                }

                // add the neighbor
                na->neighbors[na->nneighbors++] = nbidx;
            }
        }

        // shuffle delete bestnodei.
        live_nodes[best_live_node_idx] = live_nodes[nlive_nodes-1];
        nlive_nodes--;

        ordering[orderingi] = bestnodei;
    }

    for (int rowi = 0; rowi < mat->nrows; rowi++)
        free(nodes[rowi].neighbors);

    free(nodes);
    free(set);
    free(live_nodes);

    return ordering;
}
