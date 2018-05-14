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

#ifndef APRIL_GRAPH_UTIL_H
#define APRIL_GRAPH_UTIL_H

#include "april_graph.h"
#include "common/zarray.h"

// Return a list of all the connected components, with non-standard copy
// semantics. Optimizing the individual graphs WILL update the original
// graph nodes' state. Use the provided destroy function.
zarray_t * april_graph_get_connected_components(april_graph_t * graph);

void april_graph_connected_components_destroy(zarray_t * graphs);
#endif
