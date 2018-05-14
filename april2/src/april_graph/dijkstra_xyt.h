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

#ifndef _DIJKSTRA_H
#define _DIJKSTRA_H

#include "april_graph.h"

/////////////////////////////////////////////////
typedef struct dijkstra_xyt_edge dijkstra_xyt_edge_t;
struct dijkstra_xyt_edge
{
    double xyt[3];
    double cov[9];

    // a is always the refpos
    // b is the index of the node to which this edge leads.
    int a, b;
};

typedef struct dijkstra_xyt dijkstra_xyt_t;
struct dijkstra_xyt
{
    // contains a dijkstra edge for every node; entries are NULL for disconnected nodes
    dijkstra_xyt_edge_t **edges;

    // equal to the number of poses in the graph
    int nedges;

    // what reference pose index was used to generate this projection?
    int refpos;
};

dijkstra_xyt_t *dijkstra_xyt_create(april_graph_t *graph, int refpos);

void dijkstra_xyt_destroy(dijkstra_xyt_t *proj);

#endif
