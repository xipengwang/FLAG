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

#include <stdio.h>
#include "vxo_generic.h"

void vxo_generic_incref(vx_object_t *vxo)
{
    vx_lock();
    vxo->refcnt++;
    vx_unlock();
}

void vxo_generic_decref(vx_object_t *vxo)
{
    vx_lock();

    int c = --vxo->refcnt;
    vx_unlock();

    if (c == 0) {
        vxo->u.generic.program_resource->decref(vxo->u.generic.program_resource);

        for (int i = 0; i < vxo->u.generic.nattributes; i++) {
            vx_resource_t *resc = vxo->u.generic.attributes[i].resource;
            resc->decref(resc);
        }

        for (int i = 0; i < vxo->u.generic.ntextures; i++) {
            vx_resource_t *resc = vxo->u.generic.textures[i].resource;
            resc->decref(resc);
        }

        for (int i = 0; i < vxo->u.generic.nuniformfs; i++) {
            free(vxo->u.generic.uniformfs[i].name);
            free(vxo->u.generic.uniformfs[i].data);
        }

        for (int i = 0; i < vxo->u.generic.nattributes; i++)
            free(vxo->u.generic.attributes[i].name);

        for (int i = 0; i < vxo->u.generic.ntextures; i++)
            free(vxo->u.generic.textures[i].name);

        free(vxo->u.generic.uniformfs);
        free(vxo->u.generic.attributes);
        free(vxo->u.generic.textures);

        free(vxo->u.generic.draws);

        memset(vxo, 0, sizeof(struct vx_object)); // fail fast
        free(vxo);
    }
}

void vxo_generic_serialize(vx_object_t *vxo, vx_serializer_t *outs)
{
    vx_serializer_u8(outs, VX_SERIALIZER_EXECUTE_PROGRAM);
    vx_serializer_resource(outs, vxo->u.generic.program_resource);

    vx_serializer_u8(outs, vxo->u.generic.nuniformfs);  // nuniformfs
    for (int i = 0; i < vxo->u.generic.nuniformfs; i++) {
        vx_serializer_string(outs, vxo->u.generic.uniformfs[i].name);
        vx_serializer_u8(outs, vxo->u.generic.uniformfs[i].nrows);
        vx_serializer_u8(outs, vxo->u.generic.uniformfs[i].ncols);
        int n = vxo->u.generic.uniformfs[i].nrows * vxo->u.generic.uniformfs[i].ncols;
        for (int j = 0; j < n; j++)
            vx_serializer_float(outs, vxo->u.generic.uniformfs[i].data[j]);
    }

    vx_serializer_u8(outs, vxo->u.generic.nattributes);
    for (int i = 0; i < vxo->u.generic.nattributes; i++) {
        vx_serializer_string(outs, vxo->u.generic.attributes[i].name);
        vx_serializer_resource(outs, vxo->u.generic.attributes[i].resource);
    }

    vx_serializer_u8(outs, vxo->u.generic.ntextures);
    for (int i = 0; i < vxo->u.generic.ntextures; i++) {
        vx_serializer_string(outs, vxo->u.generic.textures[i].name);
        vx_serializer_resource(outs, vxo->u.generic.textures[i].resource);
    }

    vx_serializer_u8(outs, vxo->u.generic.ndraws);
    for (int i = 0; i < vxo->u.generic.ndraws; i++) {
        if (vxo->u.generic.draws[i].indices_resource) {
            vx_serializer_u8(outs, 1);
            vx_serializer_resource(outs, vxo->u.generic.draws[i].indices_resource);
        } else {
            vx_serializer_u8(outs, 0);
        }
        vx_serializer_u32(outs, vxo->u.generic.draws[i].command);
        vx_serializer_u32(outs, vxo->u.generic.draws[i].first);
        vx_serializer_u32(outs, vxo->u.generic.draws[i].count);
    }
}

// returns an object with refcnt = 0, but incref's the resources upon
// which it depends.
vx_object_t *vxo_generic_create(vx_resource_t *program_resource,
                                struct vxo_generic_uniformf  uniformfs[],
                                struct vxo_generic_attribute attributes[],
                                struct vxo_generic_texture   textures[],
                                struct vxo_generic_draw      draws[])
{
    vx_object_t *vxo = calloc(1, sizeof(vx_object_t));
    vxo->incref = vxo_generic_incref;
    vxo->decref = vxo_generic_decref;
    vxo->serialize = vxo_generic_serialize;

    vxo->u.generic.program_resource = program_resource;
    vx_resource_incref(vxo->u.generic.program_resource);

    int nuniformfs = 0, nattributes = 0, ntextures = 0, ndraws = 0;
    while (uniformfs[nuniformfs].name != NULL)
        nuniformfs++;
    while (attributes[nattributes].name != NULL)
        nattributes++;
    while (textures[ntextures].name != NULL)
        ntextures++;
    while (draws[ndraws].count > 0)
        ndraws++;

    vxo->u.generic.nuniformfs = nuniformfs;
    vxo->u.generic.nattributes = nattributes;
    vxo->u.generic.ntextures = ntextures;
    vxo->u.generic.ndraws = ndraws;

    vxo->u.generic.uniformfs = calloc(nuniformfs, sizeof(struct vxo_generic_uniformf));
    for (int i = 0; i < nuniformfs; i++) {

        vxo->u.generic.uniformfs[i].name = strdup(uniformfs[i].name);
        vxo->u.generic.uniformfs[i].nrows = uniformfs[i].nrows;
        vxo->u.generic.uniformfs[i].ncols = uniformfs[i].ncols;
        int sz = sizeof(float)*uniformfs[i].nrows*uniformfs[i].ncols;
        vxo->u.generic.uniformfs[i].data = malloc(sz);
        memcpy(vxo->u.generic.uniformfs[i].data, uniformfs[i].data, sz);
    }

    vxo->u.generic.attributes = calloc(nattributes, sizeof(struct vxo_generic_attribute));
    for (int i = 0; i < nattributes; i++) {
        vxo->u.generic.attributes[i].name = strdup(attributes[i].name);
        vxo->u.generic.attributes[i].resource = attributes[i].resource;
        vx_resource_incref(vxo->u.generic.attributes[i].resource);
    }

    vxo->u.generic.textures = calloc(ntextures, sizeof(struct vxo_generic_texture));
    for (int i = 0; i < ntextures; i++) {
        vxo->u.generic.textures[i].name = strdup(textures[i].name);
        vxo->u.generic.textures[i].resource = textures[i].resource;
        vx_resource_incref(vxo->u.generic.textures[i].resource);
    }

    vxo->u.generic.draws = calloc(ndraws, sizeof(struct vxo_generic_draw));
    for (int i = 0; i < ndraws; i++) {
        if (draws[i].indices_resource) {
            vxo->u.generic.draws[i].indices_resource = draws[i].indices_resource;
            vx_resource_incref(vxo->u.generic.draws[i].indices_resource);
        }

        vxo->u.generic.draws[i].first = draws[i].first;
        vxo->u.generic.draws[i].count = draws[i].count;
        vxo->u.generic.draws[i].command = draws[i].command;
    }

    return vxo;
}
