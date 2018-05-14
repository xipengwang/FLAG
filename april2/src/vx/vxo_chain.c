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

#include <stdarg.h>
#include <stdio.h>

#include "vx.h"

//////////////////////////////////////////////////////////
// vx chain and its cousins.

static void vxo_chain_incref(vx_object_t *vxo)
{
    vx_lock();
    vxo->refcnt++;

    vx_unlock();
}

static void vxo_chain_decref(vx_object_t *vxo)
{
    vx_lock();
    assert(vxo->refcnt > 0);
    int c = --vxo->refcnt;
    vx_unlock();

    if (c == 0) {
        for (int i = 0; i < zarray_size(vxo->u.chain.objs); i++) {
            vx_object_t *child;
            zarray_get(vxo->u.chain.objs, i, &child);
            child->decref(child);
        }

        zarray_destroy(vxo->u.chain.objs);
        memset(vxo, 0, sizeof(struct vx_object)); // fail fast
        free(vxo);
    }
}

static void vxo_chain_serialize(vx_object_t *vxo, vx_serializer_t *outs)
{
    vx_serializer_opcode(outs, VX_SERIALIZER_MODEL_PUSH);

    for (int i = 0; i < zarray_size(vxo->u.chain.objs); i++) {
        vx_object_t *child;
        zarray_get(vxo->u.chain.objs, i, &child);
        child->serialize(child, outs);
    }

    vx_serializer_opcode(outs, VX_SERIALIZER_MODEL_POP);
}

void vxo_chain_add(vx_object_t *chain, vx_object_t *obj)
{
    // type check.
    assert(chain->serialize == vxo_chain_serialize);

    if (obj == NULL)
        return;

    obj->incref(obj);
    zarray_add(chain->u.chain.objs, &obj);
}

// the basic initializer used by vxo_chain, vxo_pixcoords, vxo_depth_test, etc.
static vx_object_t *vxo_chain_v(vx_object_t *first, va_list ap)
{
    vx_object_t *vxo = calloc(1, sizeof(vx_object_t));
    vxo->u.chain.objs = zarray_create(sizeof(vx_object_t*));
    vxo->serialize = vxo_chain_serialize;
    vxo->incref = vxo_chain_incref;
    vxo->decref = vxo_chain_decref;

    if (first != NULL)
        vxo_chain_add(vxo, first);

    while (1) {
        vx_object_t *o = va_arg(ap, vx_object_t*);
        if (o == NULL)
            break;

        vxo_chain_add(vxo, o);
    }

    return vxo;
}

vx_object_t *vxo_chain(vx_object_t *first, ...)
{
    va_list ap;
    va_start(ap, first);
    vx_object_t *vxo = vxo_chain_v(first, ap);
    va_end(ap);

    return vxo;
}

static void vxo_pixcoords_serialize(vx_object_t *vxo, vx_serializer_t *outs)
{
    vx_serializer_opcode(outs, VX_SERIALIZER_MODEL_PUSH);
    vx_serializer_opcode(outs, VX_SERIALIZER_PIXCOORD_PUSH);
    vx_serializer_float(outs, vxo->u.chain.u.pixcoords.widthscale);
    vx_serializer_float(outs, vxo->u.chain.u.pixcoords.heightscale);
    vx_serializer_u8(outs, vxo->u.chain.u.pixcoords.scale_mode);

    for (int i = 0; i < zarray_size(vxo->u.chain.objs); i++) {
        vx_object_t *child;
        zarray_get(vxo->u.chain.objs, i, &child);
        child->serialize(child, outs);
    }

    vx_serializer_opcode(outs, VX_SERIALIZER_PIXCOORD_POP);
    vx_serializer_opcode(outs, VX_SERIALIZER_MODEL_POP);
}

vx_object_t *vxo_pixcoords(int origin, int scale_mode, ...)
{
    va_list ap;
    va_start(ap, scale_mode);
    vx_object_t *vxo = vxo_chain_v(NULL, ap);
    va_end(ap);

    vxo->serialize = vxo_pixcoords_serialize;

    float tx = 0, ty = 0;

    switch (origin) {
        case VXO_PIXCOORDS_BOTTOM_LEFT:
        case VXO_PIXCOORDS_BOTTOM:
        case VXO_PIXCOORDS_BOTTOM_RIGHT:
            ty = 0;
            break;

        case VXO_PIXCOORDS_LEFT:
        case VXO_PIXCOORDS_CENTER:
        case VXO_PIXCOORDS_RIGHT:
            ty = 0.5;
            break;
        case VXO_PIXCOORDS_CENTER_ROUND:
            ty = 0.5; // XXXX
            break;

        case VXO_PIXCOORDS_TOP_LEFT:
        case VXO_PIXCOORDS_TOP:
        case VXO_PIXCOORDS_TOP_RIGHT:
            ty = 1;
            break;
    }

    switch (origin) {
        case VXO_PIXCOORDS_BOTTOM_LEFT:
        case VXO_PIXCOORDS_TOP_LEFT:
        case VXO_PIXCOORDS_LEFT:
            tx = 0;
            break;

        case VXO_PIXCOORDS_BOTTOM:
        case VXO_PIXCOORDS_CENTER:
        case VXO_PIXCOORDS_TOP:
            tx = 0.5;
            break;

        case VXO_PIXCOORDS_CENTER_ROUND:
            tx = 0.5; // XXX wrong
            break;

        case VXO_PIXCOORDS_BOTTOM_RIGHT:
        case VXO_PIXCOORDS_TOP_RIGHT:
        case VXO_PIXCOORDS_RIGHT:
            tx = 1.0;
            break;
    }

    vxo->u.chain.u.pixcoords.coordspace = 1;
    vxo->u.chain.u.pixcoords.widthscale = tx;
    vxo->u.chain.u.pixcoords.heightscale = ty;
    vxo->u.chain.u.pixcoords.scale_mode = scale_mode;

    return vxo;
}

static void vxo_depth_test_serialize(vx_object_t *vxo, vx_serializer_t *outs)
{
    vx_serializer_opcode(outs, VX_SERIALIZER_MODEL_PUSH);
    vx_serializer_opcode(outs, VX_SERIALIZER_DEPTH_TEST_PUSH);
    vx_serializer_u8(outs, vxo->u.chain.u.depth_test.enable);

    for (int i = 0; i < zarray_size(vxo->u.chain.objs); i++) {
        vx_object_t *child;
        zarray_get(vxo->u.chain.objs, i, &child);
        child->serialize(child, outs);
    }

    vx_serializer_opcode(outs, VX_SERIALIZER_DEPTH_TEST_POP);
    vx_serializer_opcode(outs, VX_SERIALIZER_MODEL_POP);
}

vx_object_t *vxo_depth_test(int enable, ...)
{
    va_list ap;
    va_start(ap, enable);
    vx_object_t *vxo = vxo_chain_v(NULL, ap);
    va_end(ap);

    vxo->u.chain.u.depth_test.enable = enable;
    vxo->serialize = vxo_depth_test_serialize;

    return vxo;
}
