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

#include "common/mesh_model.h"
#include "vx.h"

vx_object_t *vxo_box_solid_shiny(float rgba[], float roughness)
{
    mesh_model_t *mm = mesh_model_create();
    struct mesh_model_chunk *chunk = mesh_model_chunk_create_box();
    zarray_add(mm->chunks, &chunk);

    memcpy(chunk->rgba, rgba, 4*sizeof(float));
    chunk->roughness = roughness;
    vx_object_t * ret = vxo_mesh_model_create(mm);
    mesh_model_destroy(mm);
    return ret;
}

vx_object_t *vxo_box_solid(float rgba[])
{
    return vxo_box_solid_shiny(rgba, 0.5);
}
