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

#ifndef _VXO_GENERIC_H
#define _VXO_GENERIC_H

#include "vx.h"

void vxo_generic_incref(vx_object_t *vxo);
void vxo_generic_decref(vx_object_t *vxo);
void vxo_generic_serialize(vx_object_t *vxo, vx_serializer_t *outs);

// This is a convenience function for initializing a generic vxo
// object. It makes a shallow copy of all the passed-in values,
// allowing the caller to use simple inline initializers for the call
// parameters. (See vxo_robot.c for an example).
//
// Recall that a newly-created object should
vx_object_t *vxo_generic_create(vx_resource_t *program_resource,
                                struct vxo_generic_uniformf uniformfs[],
                                struct vxo_generic_attribute attributes[],
                                struct vxo_generic_texture textures[],
                                struct vxo_generic_draw      draws[]);


#endif
