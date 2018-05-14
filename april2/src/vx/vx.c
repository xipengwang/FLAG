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
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <pthread.h>
#include <stdint.h>

#include "common/encode_bytes.h"

#include "vx.h"

static uint64_t vx_next_resource_id = 1;
static pthread_mutex_t vx_global_mutex;

static int is_power_two(int v)
{
    return (v & (v - 1)) == 0;
}

static int canvas_event_handler_sort(const void *_a, const void *_b)
{
    struct vx_canvas_event_handler_info *a = *((struct vx_canvas_event_handler_info**) _a);
    struct vx_canvas_event_handler_info *b = *((struct vx_canvas_event_handler_info**) _b);

    return a->dispatch_order - b->dispatch_order;
}

static int layer_event_handler_sort(const void *_a, const void *_b)
{
    struct vx_layer_event_handler_info *a = *((struct vx_layer_event_handler_info**) _a);
    struct vx_layer_event_handler_info *b = *((struct vx_layer_event_handler_info**) _b);

    return a->dispatch_order - b->dispatch_order;
}

void vx_lock()
{
    static int first = 1;

    if (first) {
        // XXX RACE on initialization.
        pthread_mutexattr_t attr;

        pthread_mutexattr_init(&attr);
        pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
        pthread_mutex_init(&vx_global_mutex, &attr);
        first = 0;
    }

    pthread_mutex_lock(&vx_global_mutex);
}

void vx_unlock()
{
    pthread_mutex_unlock(&vx_global_mutex);
}

uint64_t vx_resource_allocate_id()
{
    vx_lock();
    uint64_t v = vx_next_resource_id;
    vx_next_resource_id++;
    vx_unlock();

    return v;
}

vx_serializer_t *vx_serializer_create();

//////////////////////////////////////////////////////////
// vx_canvas
// NB: Not used by end-users.
vx_canvas_t *vx_canvas_create()
{
    vx_canvas_t *vc = calloc(1, sizeof(vx_canvas_t));

    vc->layers = zhash_create(sizeof(char*), sizeof(vx_buffer_t*),
                              zhash_str_hash, zhash_str_equals);

    vc->event_handlers = zarray_create(sizeof(struct vx_layer_event_handler_info*));

    pthread_mutex_init(&vc->mutex, NULL);
    pthread_cond_init(&vc->cond, NULL);
    vc->canvas_messages = zarray_create(sizeof(struct vx_message));

    return vc;
}

void vx_canvas_add_event_handler(vx_canvas_t *vc,
                                 int (*handler)(vx_canvas_t *vc, const vx_event_t *ev, void *user),
                                 int dispatch_order,
                                 void *user)
{
    struct vx_canvas_event_handler_info *info = calloc(1, sizeof(struct vx_canvas_event_handler_info));
    info->handler = handler;
    info->user = user;
    info->dispatch_order = dispatch_order;
    zarray_add(vc->event_handlers, &info);
    zarray_sort(vc->event_handlers, canvas_event_handler_sort);
}

void vx_canvas_dispatch_event(vx_canvas_t *vc, const vx_event_t *ev)
{
    struct vx_canvas_event_handler_info *info;
    for (int i = 0; i < zarray_size(vc->event_handlers); i++) {
        zarray_get(vc->event_handlers, i, &info);
        if (info->handler(vc, ev, info->user))
            return;
    }
}

vx_layer_t *vx_canvas_get_layer(vx_canvas_t *vc, const char *name)
{
    vx_lock();

    vx_layer_t *vl;

    if (!zhash_get(vc->layers, &name, &vl)) {
        vl = calloc(1, sizeof(vx_layer_t));
        vl->canvas = vc;
        vl->name = strdup(name);
        zhash_put(vc->layers, &vl->name, &vl, NULL, NULL);
        vl->event_handlers = zarray_create(sizeof(struct vx_layer_event_handler_info*));
    }

    vx_unlock();

    return vl;
}

//////////////////////////////////////////////////////////
// vx_layer


void vx_layer_set_world(vx_layer_t *vl, vx_world_t *vw)
{
    vx_lock();

    vx_canvas_t *vc = vl->canvas;
    vl->world = vw;

    zarray_add(vw->layers, &vl);

//    vx_layer_set_draw_order(vl, vl->draw_order);

    // mark every buffer as needing to be redrawn
    zhash_iterator_t zit;
    zhash_iterator_init(vw->buffers, &zit);
    char *name;
    vx_buffer_t *vb;
    while (zhash_iterator_next(&zit, &name, &vb)) {
        struct vx_message msg = { .type = VX_MESSAGE_BUFFER_REDRAW,
                                  .u.buffer_redraw.buffer_name = strdup(vb->name),
                                  .u.buffer_redraw.layer_name = strdup(vl->name)};
        zarray_add(vc->canvas_messages, &msg);

        vx_buffer_set_draw_order(vb, vb->draw_order);
    }

    // notify the canvas.
    pthread_mutex_lock(&vc->mutex);
    pthread_cond_broadcast(&vc->cond);
    pthread_mutex_unlock(&vc->mutex);

    vx_unlock();
}

//////////////////////////////////////////////////////////

void vx_layer_add_event_handler(vx_layer_t *vl,
                                int (*handler)(vx_layer_t *vl, const vx_event_t *ev, void *user),
                                int dispatch_order,
                                void *user)
{
    struct vx_layer_event_handler_info *info = calloc(1, sizeof(struct vx_layer_event_handler_info));
    info->handler = handler;
    info->user = user;
    info->dispatch_order = dispatch_order;
    zarray_add(vl->event_handlers, &info);
    zarray_sort(vl->event_handlers, layer_event_handler_sort);
}

void vx_layer_dispatch_event(vx_layer_t *vl, const vx_event_t *ev)
{
    struct vx_layer_event_handler_info *info;
    for (int i = 0; i < zarray_size(vl->event_handlers); i++) {
        zarray_get(vl->event_handlers, i, &info);
        if (info->handler(vl, ev, info->user))
            return;
    }
}


//////////////////////////////////////////////////////////
// vx_world
vx_world_t *vx_world_create()
{
    vx_world_t *vw = calloc(1, sizeof(vx_world_t));

    vw->buffers = zhash_create(sizeof(char*), sizeof(vx_buffer_t*),
                               zhash_str_hash, zhash_str_equals);

    vw->layers = zarray_create(sizeof(vx_layer_t*));
    return vw;
}

void vx_world_destroy_buffer(vx_world_t *vw, const char *name)
{
    vx_buffer_t *vb;

    vx_lock();

    if (!zhash_get(vw->buffers, &name, &vb))
        goto cleanup;

    for (int i = 0; i < zarray_size(vb->world->layers); i++) {
        vx_layer_t *vl;
        zarray_get(vb->world->layers, i, &vl);

        vx_canvas_t *vc = vl->canvas;

        pthread_mutex_lock(&vc->mutex);

        // enqueue us to be serialized
        struct vx_message msg = { .type = VX_MESSAGE_BUFFER_DESTROY,
                                  .u.buffer_destroy.layer_name = strdup(vl->name),
                                  .u.buffer_destroy.buffer_name = strdup(vb->name) };

        zarray_add(vc->canvas_messages, &msg);
        pthread_cond_broadcast(&vc->cond);

        pthread_mutex_unlock(&vc->mutex);
    }


    char *old_key;
    vx_buffer_t *old_buffer;
    if (zhash_remove(vw->buffers, &name, &old_key, &old_buffer)) {
        assert(old_buffer == vb);
        free(old_key);

        vx_serializer_destroy(vb->serializer_front);
        vx_serializer_destroy(vb->serializer_back);
        free(old_buffer);
    }

  cleanup:
    vx_unlock();
}

// if create is 1, creates a new buffer automatically.
vx_buffer_t *vx_world_get_buffer_x(vx_world_t *vw, const char *name, int create)
{
    vx_buffer_t *vb = NULL;

    vx_lock();

    if (!zhash_get(vw->buffers, &name, &vb) && create) {
        vb = calloc(1, sizeof(vx_buffer_t));
        vb->name = strdup(name);
        vb->world = vw;
        zhash_put(vw->buffers, &vb->name, &vb, NULL, NULL);
        vb->serializer_front = vx_serializer_create();
        vb->serializer_back = vx_serializer_create();
    }

    vx_unlock();

    return vb;
}

vx_buffer_t *vx_world_get_buffer(vx_world_t *vw, const char *name)
{
    return vx_world_get_buffer_x(vw, name, 1);
}

static void generic_resource_incref(vx_resource_t *resc)
{
    vx_lock();
    resc->refcnt++;
    vx_unlock();
}

//////////////////////////////////////////////////////////////
static void program_resource_decref(vx_resource_t *resc)
{
    vx_lock(); do {

        assert(resc->refcnt > 0);
        resc->refcnt--;

        if (resc->refcnt != 0)
            break;

        free(resc->u.program.vertex_shader_src);
        free(resc->u.program.fragment_shader_src);
        free(resc);
    } while (0);

    vx_unlock();
}

vx_resource_t *vx_resource_make_program(const char *vertex_shader_src, const char *fragment_shader_src)
{
    vx_resource_t *resc = calloc(1, sizeof(vx_resource_t));
    resc->type = VX_RESOURCE_PROGRAM;
    resc->id = vx_resource_allocate_id();
    resc->u.program.vertex_shader_src = strdup(vertex_shader_src);
    resc->u.program.fragment_shader_src = strdup(fragment_shader_src);
    resc->incref = generic_resource_incref;
    resc->decref = program_resource_decref;
    return resc;
}

//////////////////////////////////////////////////////////////
static void attr_f32_resource_decref(vx_resource_t *resc)
{
    vx_lock(); do {

        assert(resc->refcnt > 0);
        resc->refcnt--;

        if (resc->refcnt != 0)
            break;

        free(resc->u.attr_f32.data);
        free(resc);
    } while (0);

    vx_unlock();
}

vx_resource_t *vx_resource_make_attr_f32_copy(const float *data, int nelements, int dim)
{
    vx_resource_t *resc = calloc(1, sizeof(vx_resource_t));

    resc->type = VX_RESOURCE_ATTR_F32;
    resc->id = vx_resource_allocate_id();
    resc->u.attr_f32.data = malloc(nelements * sizeof(float));
    memcpy(resc->u.attr_f32.data, data, nelements * sizeof(float));
    resc->u.attr_f32.nelements = nelements;
    resc->u.attr_f32.dim = dim;
    resc->incref = generic_resource_incref;
    resc->decref = attr_f32_resource_decref;
    return resc;
}

//////////////////////////////////////////////////////////////
static void idx_u16_resource_decref(vx_resource_t *resc)
{
    vx_lock(); do {

        assert(resc->refcnt > 0);
        resc->refcnt--;

        if (resc->refcnt != 0)
            break;

        free(resc->u.idx_u16.data);
        free(resc);
    } while (0);

    vx_unlock();
}

vx_resource_t *vx_resource_make_idx_u16_copy(const uint16_t *data, int nelements)
{
    vx_resource_t *resc = calloc(1, sizeof(vx_resource_t));

    resc->type = VX_RESOURCE_IDX_U16;
    resc->id = vx_resource_allocate_id();
    resc->u.idx_u16.data = malloc(nelements * sizeof(uint16_t));
    memcpy(resc->u.idx_u16.data, data, nelements * sizeof(uint16_t));
    resc->u.idx_u16.nelements = nelements;
    resc->incref = generic_resource_incref;
    resc->decref = idx_u16_resource_decref;
    return resc;
}

//////////////////////////////////////////////////////////////
static void texture_u8x3_resource_decref(vx_resource_t *resc)
{
    vx_lock(); do {

        assert(resc->refcnt > 0);
        resc->refcnt--;

        if (resc->refcnt != 0)
            break;

        image_u8x3_destroy(resc->u.texture_u8x3.im);
        free(resc);
    } while (0);

    vx_unlock();
}

vx_resource_t *vx_resource_make_texture_u8x3_copy(const image_u8x3_t *im, int flags)
{
    if ((!is_power_two(im->width) || !is_power_two(im->height)) &&
        (flags & (VX_TEXTURE_WRAP | VX_TEXTURE_MIPMAP))) {
        printf("WARNING: non-power of two image is using unsafe options.\n");
    }

    vx_resource_t *resc = calloc(1, sizeof(vx_resource_t));

    resc->type = VX_RESOURCE_TEXTURE_U8X3;
    resc->id = vx_resource_allocate_id();
    resc->u.texture_u8x3.im = image_u8x3_copy(im);
    resc->u.texture_u8x3.flags = flags;
    resc->incref = generic_resource_incref;
    resc->decref = texture_u8x3_resource_decref;
    return resc;
}

//////////////////////////////////////////////////////////////
static void texture_u8x4_resource_decref(vx_resource_t *resc)
{
    vx_lock(); do {

        assert(resc->refcnt > 0);
        resc->refcnt--;

        if (resc->refcnt != 0)
            break;

        image_u8x4_destroy(resc->u.texture_u8x4.im);
        free(resc);
    } while (0);

    vx_unlock();
}

vx_resource_t *vx_resource_make_texture_u8x4_copy(const image_u8x4_t *im, int flags)
{
    if ((!is_power_two(im->width) || !is_power_two(im->height)) &&
        (flags & (VX_TEXTURE_WRAP | VX_TEXTURE_MIPMAP))) {
        printf("WARNING: non-power of two image is using unsafe options.\n");
    }

    vx_resource_t *resc = calloc(1, sizeof(vx_resource_t));

    resc->type = VX_RESOURCE_TEXTURE_U8X4;
    resc->id = vx_resource_allocate_id();
    resc->u.texture_u8x4.im = image_u8x4_copy(im);
    resc->u.texture_u8x4.flags = flags;
    resc->incref = generic_resource_incref;
    resc->decref = texture_u8x4_resource_decref;
    return resc;
}

//////////////////////////////////////////////////////////////
vx_resource_t *vx_resource_make_texture_pam_copy(const pam_t *pam, int flags)
{
    if (pam->depth == 1) {
        image_u8_t *im = image_u8_create(pam->width, pam->height);
        for (int y = 0; y < im->height; y++)
            memcpy(&im->buf[y*im->stride], &pam->data[y*pam->width*pam->depth], pam->width * pam->depth);
        vx_resource_t *resc = vx_resource_make_texture_u8_copy(im, flags);
        image_u8_destroy(im);
        return resc;
    }

    if (pam->depth == 3) {
        image_u8x3_t *im = image_u8x3_create(pam->width, pam->height);
        for (int y = 0; y < im->height; y++)
            memcpy(&im->buf[y*im->stride], &pam->data[y*pam->width*pam->depth], pam->width * pam->depth);
        vx_resource_t *resc = vx_resource_make_texture_u8x3_copy(im, flags);
        image_u8x3_destroy(im);
        return resc;
    }

    if (pam->depth == 4) {
        image_u8x4_t *im = image_u8x4_create(pam->width, pam->height);
        for (int y = 0; y < im->height; y++)
            memcpy(&im->buf[y*im->stride], &pam->data[y*pam->width*pam->depth], pam->width * pam->depth);
        vx_resource_t *resc = vx_resource_make_texture_u8x4_copy(im, flags);
        image_u8x4_destroy(im);
        return resc;
    }

    assert(0);
    return NULL;
}

//////////////////////////////////////////////////////////////
static void texture_u8_resource_decref(vx_resource_t *resc)
{
    vx_lock(); do {

        assert(resc->refcnt > 0);
        resc->refcnt--;

        if (resc->refcnt != 0)
            break;

        image_u8_destroy(resc->u.texture_u8.im);
        free(resc);
    } while (0);

    vx_unlock();
}

vx_resource_t *vx_resource_make_texture_u8_copy(const image_u8_t *im, int flags)
{
    if ((!is_power_two(im->width) || !is_power_two(im->height)) &&
        (flags & (VX_TEXTURE_WRAP | VX_TEXTURE_MIPMAP))) {
        printf("WARNING: non-power of two image is using unsafe options.\n");
    }

    vx_resource_t *resc = calloc(1, sizeof(vx_resource_t));

    resc->type = VX_RESOURCE_TEXTURE_U8;
    resc->id = vx_resource_allocate_id();
    resc->u.texture_u8.im = image_u8_copy(im);
    resc->u.texture_u8.flags = flags;
    resc->incref = generic_resource_incref;
    resc->decref = texture_u8_resource_decref;
    return resc;
}

//////////////////////////////////////////////////////////////
static void vx_serializer_ensure_space(vx_serializer_t *outs, int space)
{
    if (outs->data_len + space < outs->data_alloc)
        return;

    outs->data_alloc *= 2;
    outs->data = realloc(outs->data, outs->data_alloc);
}

static void vx_resource_invoke_incref(vx_resource_t *resc)
{
    resc->incref(resc);
}

static void vx_resource_invoke_decref(vx_resource_t *resc)
{
    resc->decref(resc);
}

// copies the encoded byte stream and increment the reference count
vx_serializer_t *vx_serializer_copy(vx_serializer_t *in)
{
    vx_serializer_t *copy = calloc(1, sizeof(vx_serializer_t));

    copy->resources = zset_copy(in->resources);
    zset_vmap(copy->resources, vx_resource_invoke_incref);

    copy->data_alloc = in->data_len;
    copy->data = malloc(copy->data_alloc);
    memcpy(copy->data, in->data, in->data_len);
    copy->data_len = in->data_len;

    return copy;
}

vx_serializer_t *vx_serializer_create()
{
    vx_serializer_t *outs = calloc(1, sizeof(vx_serializer_t));
    outs->data_alloc = 4096;
    outs->data = calloc(1, outs->data_alloc);

    outs->resources = zset_create(sizeof(vx_resource_t*),
                                  zhash_ptr_hash, zhash_ptr_equals);

    return outs;
}

void vx_serializer_u8(vx_serializer_t *outs, uint8_t v)
{
    vx_serializer_ensure_space(outs, 1);

    outs->data[outs->data_len++] = v;
}

void vx_serializer_u32(vx_serializer_t *outs, uint32_t v)
{
    vx_serializer_ensure_space(outs, 4);

    outs->data[outs->data_len++] = (v >> 24) & 0xff;
    outs->data[outs->data_len++] = (v >> 16) & 0xff;
    outs->data[outs->data_len++] = (v >> 8) & 0xff;
    outs->data[outs->data_len++] = (v >> 0) & 0xff;
}

void vx_serializer_u64(vx_serializer_t *outs, uint64_t v)
{
    vx_serializer_ensure_space(outs, 8);

    outs->data[outs->data_len++] = (v >> 56) & 0xff;
    outs->data[outs->data_len++] = (v >> 48) & 0xff;
    outs->data[outs->data_len++] = (v >> 40) & 0xff;
    outs->data[outs->data_len++] = (v >> 32) & 0xff;
    outs->data[outs->data_len++] = (v >> 24) & 0xff;
    outs->data[outs->data_len++] = (v >> 16) & 0xff;
    outs->data[outs->data_len++] = (v >> 8) & 0xff;
    outs->data[outs->data_len++] = (v >> 0) & 0xff;
}

void vx_serializer_double(vx_serializer_t *outs, double v)
{
    union {
        double dv;
        uint64_t uv;
    } u;

    u.dv = v;

    vx_serializer_u64(outs, u.uv);
}

void vx_serializer_float(vx_serializer_t *outs, float v)
{
    union {
        float fv;
        uint32_t uv;
    } u;

    u.fv = v;

    vx_serializer_u32(outs, u.uv);
}

void vx_serializer_opcode(vx_serializer_t *outs, uint8_t opcode)
{
    vx_serializer_u8(outs, opcode);
}

void vx_serializer_resource(vx_serializer_t *outs, vx_resource_t *resc)
{
    zset_add(outs->resources, &resc, NULL);
    resc->incref(resc);

    vx_serializer_u64(outs, resc->id);
}

void vx_serializer_string(vx_serializer_t *outs, const char *s)
{
    if (s == NULL) {
        vx_serializer_u32(outs, 0x7fffffff);
        return;
    }

    int len = strlen(s);
    vx_serializer_u32(outs, len);

    if (1) {
        vx_serializer_ensure_space(outs, len);
        memcpy(&outs->data[outs->data_len], s, len);
        outs->data_len += len;
    } else {
        // XXX inefficient
        for (int i = 0; i < len; i++) {
            vx_serializer_u8(outs, s[i]);
        }
    }
}

// will decrement the reference counts of the resources.
void vx_serializer_destroy(vx_serializer_t *outs)
{
    if (!outs)
        return;

    zset_vmap(outs->resources, vx_resource_invoke_decref);
    zset_destroy(outs->resources);
    free(outs->data);
    free(outs);
}


//////////////////////////////////////////////////////////
// varag acts like a vx_chain
void vx_buffer_add_back(vx_buffer_t *vb, ...)
{
    vx_lock();

    va_list ap;

    if (vb->serializer_back == NULL)
        vb->serializer_back = vx_serializer_create();

    // if an object with refcnt=0 is passed in twice, it should not be
    // destroyed when the first occurence is processed. Avoid this by
    // incref'ing everything before processing.
    va_start(ap, vb);
    while (1) {
        vx_object_t *o = va_arg(ap, vx_object_t*);
        if (o == NULL)
            break;

        o->incref(o);
    }
    va_end(ap);

    vx_serializer_opcode(vb->serializer_back, VX_SERIALIZER_MODEL_PUSH);

    va_start(ap, vb);
    while (1) {
        vx_object_t *o = va_arg(ap, vx_object_t*);
        if (o == NULL)
            break;

        o->serialize(o, vb->serializer_back);
        o->decref(o);
    }

    va_end(ap);

    vx_serializer_opcode(vb->serializer_back, VX_SERIALIZER_MODEL_POP);
    vx_unlock();
}

void vx_buffer_swap(vx_buffer_t *vb)
{
    vx_lock();

    vx_serializer_destroy(vb->serializer_front);
    vb->serializer_front = vb->serializer_back;
    vb->serializer_back = vx_serializer_create();

    // tell all the layers that use this world that we're dirty and
    // need to be redrawn
    for (int i = 0; i < zarray_size(vb->world->layers); i++) {
        vx_layer_t *vl;
        zarray_get(vb->world->layers, i, &vl);

        vx_canvas_t *vc = vl->canvas;

        pthread_mutex_lock(&vc->mutex);

        for (int j = 0; j < zarray_size(vc->canvas_messages); j++) {
            struct vx_message msg;
            zarray_get(vc->canvas_messages, j, &msg);

            // this buffer already scheduled to be serialized?
            if (msg.type == VX_MESSAGE_BUFFER_REDRAW &&
                !strcmp(msg.u.buffer_redraw.layer_name, vl->name) &&
                !strcmp(msg.u.buffer_redraw.buffer_name, vb->name))
                goto done;
        }

        // enqueue us to be serialized
        struct vx_message msg = { .type = VX_MESSAGE_BUFFER_REDRAW,
                                  .u.buffer_redraw.layer_name = strdup(vl->name),
                                  .u.buffer_redraw.buffer_name = strdup(vb->name) };

        zarray_add(vc->canvas_messages, &msg);
        pthread_cond_broadcast(&vc->cond);

      done: ;
        pthread_mutex_unlock(&vc->mutex);
    }

    vx_unlock();
}

static void vx_message_post(vx_canvas_t *vc, struct vx_message *msg)
{
    pthread_mutex_lock(&vc->mutex);
    zarray_add(vc->canvas_messages, msg);
    pthread_cond_broadcast(&vc->cond);
    pthread_mutex_unlock(&vc->mutex);
}

void vx_canvas_readpixels(vx_canvas_t *vc, int64_t id)
{
    struct vx_message msg = { .type = VX_MESSAGE_CANVAS_READ_PIXELS,
                              .u.canvas_readpixels.id = id };

    vx_message_post(vc, &msg);
}

void vx_canvas_set_size(vx_canvas_t *vc, uint32_t width, uint32_t height)
{
    struct vx_message msg = { .type = VX_MESSAGE_CANVAS_SET_SIZE,
                              .u.canvas_setsize.width = width,
                              .u.canvas_setsize.height = height };

    vx_message_post(vc, &msg);
}

void vx_canvas_set_title(vx_canvas_t *vc, const char *name)
{
    struct vx_message msg = { .type = VX_MESSAGE_CANVAS_SET_TITLE,
                              .u.canvas_set_title.title = strdup(name) };

    vx_message_post(vc, &msg);
}

void vx_layer_enable_camera_controls(vx_layer_t *vl, uint32_t mask)
{
    struct vx_message msg = { .type = VX_MESSAGE_LAYER_ENABLE_CAMERA_CONTROLS,
                              .u.layer_enable_camera_controls.layer_name = strdup(vl->name),
                              .u.layer_enable_camera_controls.mask = mask };

    vx_message_post(vl->canvas, &msg);
}

void vx_layer_set_camera_mode(vx_layer_t *vl, const char *mode)
{
    struct vx_message msg = { .type = VX_MESSAGE_LAYER_SET_CAMERA_MODE,
                              .u.layer_set_camera_mode.layer_name = strdup(vl->name),
                              .u.layer_set_camera_mode.mode = strdup(mode) };

    vx_message_post(vl->canvas, &msg);
}

void vx_layer_set_background_rgba(vx_layer_t *vl, float rgba[4], float mtime)
{
    struct vx_message msg = { .type = VX_MESSAGE_LAYER_SET_BACKGROUND_COLOR,
                              .u.layer_set_background_color.layer_name = strdup(vl->name),
                              .u.layer_set_background_color.rgba[0] = rgba[0],
                              .u.layer_set_background_color.rgba[1] = rgba[1],
                              .u.layer_set_background_color.rgba[2] = rgba[2],
                              .u.layer_set_background_color.rgba[3] = rgba[3],
                              .u.layer_set_background_color.mtime = mtime};

    vx_message_post(vl->canvas, &msg);
}

void vx_layer_set_draw_order(vx_layer_t *vl, float draw_order)
{
    struct vx_message msg = { .type = VX_MESSAGE_LAYER_SET_DRAW_ORDER,
                              .u.layer_set_draw_order.layer_name = strdup(vl->name),
                              .u.layer_set_draw_order.draw_order = draw_order };

    vx_message_post(vl->canvas, &msg);
}

void vx_layer_set_position(vx_layer_t *vl, float position[4], float mtime)
{
    struct vx_message msg = { .type = VX_MESSAGE_LAYER_SET_POSITION,
                              .u.layer_set_position.layer_name = strdup(vl->name),
                              .u.layer_set_position.position[0] = position[0],
                              .u.layer_set_position.position[1] = position[1],
                              .u.layer_set_position.position[2] = position[2],
                              .u.layer_set_position.position[3] = position[3],
                              .u.layer_set_position.mtime = mtime };

    vx_message_post(vl->canvas, &msg);
}

void vx_buffer_set_draw_order(vx_buffer_t *vb, float draw_order)
{
    vb->draw_order = draw_order;

    vx_lock();

    // have to send this to every layer that uses this world
    for (int i = 0; i < zarray_size(vb->world->layers); i++) {
        vx_layer_t *vl;
        zarray_get(vb->world->layers, i, &vl);

        struct vx_message msg = { .type = VX_MESSAGE_BUFFER_REDRAW,
                                  .u.buffer_redraw.buffer_name = strdup(vb->name),
                                  .u.buffer_redraw.layer_name = strdup(vl->name)};

        vx_message_post(vl->canvas, &msg);
    }

    vx_unlock();
}

void vx_buffer_set_visible(vx_buffer_t *vb, uint32_t visible)
{
    printf("ERR: Set visible not implemented\n");
}


void vx_canvas_echo(vx_canvas_t *vc, double nonce)
{
    vx_lock();
    struct vx_message msg = { .type = VX_MESSAGE_CANVAS_ECHO,
                              .u.canvas_echo.nonce = nonce };

    vx_message_post(vc, &msg);
    vx_unlock();
}


void vx_world_set_elu(vx_world_t *vw, double eye[3], double lookat[3], double up[3], float mtime)
{
    vx_lock();

    // have to send this to every layer that uses this world
    for (int i = 0; i < zarray_size(vw->layers); i++) {
        vx_layer_t *vl;
        zarray_get(vw->layers, i, &vl);
        vx_layer_set_elu(vl, eye, lookat, up, mtime);
    }

    vx_unlock();
}

void vx_layer_set_elu(vx_layer_t *vl, double eye[3], double lookat[3], double up[3], float mtime)
{
    struct vx_message msg = { .type = VX_MESSAGE_LAYER_SET_ELU,
                              .u.layer_set_elu.layer_name = strdup(vl->name),
                              .u.layer_set_elu.eye[0] = eye[0],
                              .u.layer_set_elu.eye[1] = eye[1],
                              .u.layer_set_elu.eye[2] = eye[2],
                              .u.layer_set_elu.lookat[0] = lookat[0],
                              .u.layer_set_elu.lookat[1] = lookat[1],
                              .u.layer_set_elu.lookat[2] = lookat[2],
                              .u.layer_set_elu.up[0] = up[0],
                              .u.layer_set_elu.up[1] = up[1],
                              .u.layer_set_elu.up[2] = up[2],
                              .u.layer_set_elu.mtime = mtime };

    vx_message_post(vl->canvas, &msg);
}

void vx_util_unproject(const double winxyz[3], matd_t *P, matd_t *V, const float viewport[4], double objxyz[3])
{
    matd_t *invPV = matd_op("(M*M)^-1", P, V);

    matd_t *v = matd_create_data(4, 1, (double[]) {
                2*(winxyz[0]-viewport[0]) / viewport[2] - 1,
                2*(winxyz[1]-viewport[1]) / viewport[3] - 1,
                2*winxyz[2] - 1,
                    1 });

    matd_t *obj = matd_op("M*M", invPV, v);
    objxyz[0] = MATD_EL(obj, 0, 0) / MATD_EL(obj, 3, 0);
    objxyz[1] = MATD_EL(obj, 1, 0) / MATD_EL(obj, 3, 0);
    objxyz[2] = MATD_EL(obj, 2, 0) / MATD_EL(obj, 3, 0);

    matd_destroy(invPV);
    matd_destroy(v);
    matd_destroy(obj);
}

void vx_util_mouse_event_compute_ray(const vx_event_t *ev, double r0[3], double r1[3])
{
    double x = ev->u.mouse_button.x - ev->viewport[0];
    double y = ev->u.mouse_button.y - ev->viewport[1];

    matd_t *P = matd_create_data(4, 4, ev->P);
    matd_t *V = matd_create_data(4, 4, ev->V);

    vx_util_unproject((double[]) { x, ev->viewport[3] - y, 0 }, P, V, ev->viewport, r0);
    vx_util_unproject((double[]) { x, ev->viewport[3] - y, 1 }, P, V, ev->viewport, r1);

    matd_destroy(P);
    matd_destroy(V);
}

void vx_util_touch_event_compute_ray(const vx_event_t *ev, double r0[3], double r1[3])
{
    double x = ev->u.touch.x - ev->viewport[0];
    double y = ev->u.touch.y - ev->viewport[1];

    matd_t *P = matd_create_data(4, 4, ev->P);
    matd_t *V = matd_create_data(4, 4, ev->V);

    vx_util_unproject((double[]) { x, ev->viewport[3] - y, 0 }, P, V, ev->viewport, r0);
    vx_util_unproject((double[]) { x, ev->viewport[3] - y, 1 }, P, V, ev->viewport, r1);

    matd_destroy(P);
    matd_destroy(V);
}

void vx_util_ray_intersect_plane(const double r0[3], const double r1[3], const double ABCD[4], double xyz[3])
{
    double A = ABCD[0], B = ABCD[1], C = ABCD[2], D = ABCD[3];

    double dir[3];
    for (int i = 0; i < 3; i++)
        dir[i] = r1[i] - r0[i];

    // for every lambda=1 we move in direction 'dir', how much does the RHS change?
    double dirdot = dir[0]*A + dir[1]*B + dir[2]*C;

    // how much change do we need to achieve on the RHS?
    double dist0 = D - (r0[0]*A + r0[1]*B + r0[2]*C);

    double lambda = dist0 / dirdot;

    xyz[0] = r0[0] + lambda*dir[0];
    xyz[1] = r0[1] + lambda*dir[1];
    xyz[2] = r0[2] + lambda*dir[2];
}
