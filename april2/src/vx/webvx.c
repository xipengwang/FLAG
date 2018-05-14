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
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <stdint.h>
#include <inttypes.h>
#include <sys/types.h>
#include <netinet/tcp.h>
#include <sys/socket.h>

#include "httpd/httpd.h"
#include "httpd/httpd_websocket.h"
#include "common/april_util.h"
#include "common/zarray.h"
#include "common/zhash.h"
#include "common/sha1.h"
#include "common/encode_bytes.h"
#include "common/matd_coords.h"
#include "common/time_util.h"
#include "common/string_util.h"
#include "common/c5.h"

#include "vx.h"
#include "webvx.h"

// one of these is created for every webvx_define_canvas call.
struct webvx_canvas_definition
{
    webvx_t *webvx;
    websocket_server_t *wsserver;

    void (*on_create_canvas)(vx_canvas_t *vc, const char *name, void *impl);
    void (*on_destroy_canvas)(vx_canvas_t *vc, void *impl);
    void *impl;

    zarray_t *client_infos; // webvx_canvas_client_info*
};

// one is created for every remotely connected canvas.
struct webvx_canvas_client_info
{
    struct webvx_canvas_definition *canvasdef;
    websocket_client_t *wsclient;
    webvx_t *webvx;
    vx_canvas_t *vc;

    // protected by vx_lock
    int closed;
};


void websocket_client_send_wrapper(const char *layerbuffer, const char *type, websocket_client_t *client, uint8_t *buf, uint32_t bufpos)
{
//    printf("[ %15.3f websocket send %30s %10s: %8d ]\n", utime_now() / 1.0E6, layerbuffer, type, bufpos);
    websocket_client_send(client, buf, bufpos);
}

// This thread only writes to the socket. It ignores write errors due
// to socket close states, exiting only when wsclientinfo->closed is
// set. (This will be set by on_disconnect, which is called by the
// websocket reader thread.)
static void *webvx_canvas_thread(void *_impl)
{
    struct webvx_canvas_client_info *clientinfo = _impl;
//    webvx_t *webvx = clientinfo->webvx;
    websocket_client_t *client = clientinfo->wsclient;
    struct webvx_canvas_definition *canvasdef = clientinfo->canvasdef;

    // how many buffer+layers reference a resource? (When this count goes to zero,
    // we can instruct the remote display to discard.)
    zhash_t *remote_resources_refcnt = zhash_create(sizeof(uint64_t), sizeof(uint32_t),
                                                    zhash_uint64_hash, zhash_uint64_equals);

    // keep track of the resources that each buffer/layer used last
    // time. This allows us to determine whether a resource was
    // previously referenced or not.
    zhash_t *last_snapshots = zhash_create(sizeof(char*), sizeof(vx_serializer_t*),
                                           zhash_str_hash, zhash_str_equals);

    vx_canvas_t *vc = vx_canvas_create();
    clientinfo->vc = vc;

    canvasdef->on_create_canvas(vc, "", canvasdef->impl);

    while (1) {

        if (1) {
            vx_lock();
            int c = clientinfo->closed;
            vx_unlock();
            if (c)
                break;
        }

        //////////////////////////////////////////////////////
        // retrieve one vx_dirty object, waiting if necessary.
        pthread_mutex_lock(&vc->mutex);

//        printf("layer messages %d\n", zarray_size(vc->canvas_messages));

        if (zarray_size(vc->canvas_messages) == 0) {
            struct timespec ts;
            utime_to_timespec(utime_now() + 1E6, &ts);
            pthread_cond_timedwait(&vc->cond, &vc->mutex, &ts);
        }

        if (zarray_size(vc->canvas_messages) == 0) {
            pthread_mutex_unlock(&vc->mutex);

            // generate a NOP. This serves as a keep-alive/heartbeat.

            uint8_t buf[1024];
            uint32_t bufpos = 0;

            encode_u32(buf, &bufpos, 0x124578ab); // magic
            encode_u8(buf, &bufpos, VX_MESSAGE_NOP); // layer command

            websocket_client_sendv(client, 1, (const void*[]) { buf }, (uint64_t[]) { bufpos });
            continue;
        }

        struct vx_message msg;
        zarray_get(vc->canvas_messages, 0, &msg);
        zarray_remove_index(vc->canvas_messages, 0, 0);

        pthread_mutex_unlock(&vc->mutex);

        switch (msg.type)
        {
            case VX_MESSAGE_CANVAS_ECHO:
            {
                uint8_t buf[1024];
                uint32_t bufpos = 0;

                encode_u32(buf, &bufpos, 0x124578ab); // magic
                encode_u8(buf, &bufpos, VX_MESSAGE_CANVAS_ECHO);

                encode_f64(buf, &bufpos, msg.u.canvas_echo.nonce);

                websocket_client_sendv(client, 1,
                                       (const void*[]) { buf },
                                       (uint64_t[]) { bufpos });
                break;
            }

            case VX_MESSAGE_CANVAS_READ_PIXELS:
            {
                uint8_t buf[1024];
                uint32_t bufpos = 0;

                encode_u32(buf, &bufpos, 0x124578ab); // magic
                encode_u8(buf, &bufpos, VX_MESSAGE_CANVAS_READ_PIXELS);

                encode_u64(buf, &bufpos, msg.u.canvas_readpixels.id);

                websocket_client_sendv(client, 1,
                                       (const void*[]) { buf },
                                       (uint64_t[]) { bufpos });

                break;
            }

            case VX_MESSAGE_CANVAS_SET_SIZE:
            {
                uint8_t buf[1024];
                uint32_t bufpos = 0;

                encode_u32(buf, &bufpos, 0x124578ab); // magic
                encode_u8(buf, &bufpos, VX_MESSAGE_CANVAS_SET_SIZE); // layer command
                encode_u32(buf, &bufpos, msg.u.canvas_setsize.width);
                encode_u32(buf, &bufpos, msg.u.canvas_setsize.height);

                websocket_client_sendv(client, 1,
                                       (const void*[]) { buf },
                                       (uint64_t[]) { bufpos });
                break;
            }

            case VX_MESSAGE_CANVAS_SET_TITLE:
            {
                uint8_t buf[1024];
                uint32_t bufpos = 0;

                encode_u32(buf, &bufpos, 0x124578ab); // magic
                encode_u8(buf, &bufpos, VX_MESSAGE_CANVAS_SET_TITLE);

                encode_string_u32(buf, &bufpos, msg.u.canvas_set_title.title);

                websocket_client_sendv(client, 1,
                                       (const void*[]) { buf },
                                       (uint64_t[]) { bufpos });
                break;
            }

            case VX_MESSAGE_LAYER_ENABLE_CAMERA_CONTROLS:
            {
                uint8_t buf[1024];
                uint32_t bufpos = 0;

                encode_u32(buf, &bufpos, 0x124578ab); // magic
                encode_u8(buf, &bufpos, VX_MESSAGE_LAYER_ENABLE_CAMERA_CONTROLS);

                encode_string_u32(buf, &bufpos, msg.u.layer_enable_camera_controls.layer_name);
                encode_u32(buf, &bufpos, msg.u.layer_enable_camera_controls.mask);

                websocket_client_sendv(client, 1,
                                       (const void*[]) { buf },
                                       (uint64_t[]) { bufpos });
                break;
            }

            case VX_MESSAGE_LAYER_SET_DRAW_ORDER:
            {
                uint8_t buf[1024];
                uint32_t bufpos = 0;

                encode_u32(buf, &bufpos, 0x124578ab); // magic
                encode_u8(buf, &bufpos, VX_MESSAGE_LAYER_SET_DRAW_ORDER);

                encode_string_u32(buf, &bufpos, msg.u.layer_set_draw_order.layer_name);
                encode_f32(buf, &bufpos, msg.u.layer_set_draw_order.draw_order);

                websocket_client_sendv(client, 1,
                                       (const void*[]) { buf },
                                       (uint64_t[]) { bufpos });
                break;
            }

            case VX_MESSAGE_LAYER_SET_BACKGROUND_COLOR:
            {
                uint8_t buf[1024];
                uint32_t bufpos = 0;

                encode_u32(buf, &bufpos, 0x124578ab); // magic
                encode_u8(buf, &bufpos, VX_MESSAGE_LAYER_SET_BACKGROUND_COLOR);

                encode_string_u32(buf, &bufpos, msg.u.layer_set_background_color.layer_name);
                for (int i = 0; i < 4; i++)
                    encode_f32(buf, &bufpos, msg.u.layer_set_background_color.rgba[i]);
                encode_f32(buf, &bufpos, msg.u.layer_set_background_color.mtime);
                websocket_client_sendv(client, 1,
                                       (const void*[]) { buf },
                                       (uint64_t[]) { bufpos });
                break;
                break;
            }

            case VX_MESSAGE_LAYER_SET_POSITION:
            {
                uint8_t buf[1024];
                uint32_t bufpos = 0;

                encode_u32(buf, &bufpos, 0x124578ab); // magic
                encode_u8(buf, &bufpos, VX_MESSAGE_LAYER_SET_ELU);

                encode_string_u32(buf, &bufpos, msg.u.layer_set_position.layer_name);
                for (int i = 0; i < 4; i++)
                    encode_f32(buf, &bufpos, msg.u.layer_set_position.position[i]);
                encode_f32(buf, &bufpos, msg.u.layer_set_position.mtime);

                websocket_client_sendv(client, 1,
                                       (const void*[]) { buf },
                                       (uint64_t[]) { bufpos });
                break;
            }

            case VX_MESSAGE_LAYER_SET_ELU:
            {
                uint8_t buf[1024];
                uint32_t bufpos = 0;

                encode_u32(buf, &bufpos, 0x124578ab); // magic
                encode_u8(buf, &bufpos, VX_MESSAGE_LAYER_SET_ELU);

                encode_string_u32(buf, &bufpos, msg.u.layer_set_elu.layer_name);

                for (int i = 0; i < 3; i++)
                    encode_f64(buf, &bufpos, msg.u.layer_set_elu.eye[i]);
                for (int i = 0; i < 3; i++)
                    encode_f64(buf, &bufpos, msg.u.layer_set_elu.lookat[i]);
                for (int i = 0; i < 3; i++)
                    encode_f64(buf, &bufpos, msg.u.layer_set_elu.up[i]);
                encode_f32(buf, &bufpos, msg.u.layer_set_elu.mtime);

                websocket_client_sendv(client, 1,
                                       (const void*[]) { buf },
                                       (uint64_t[]) { bufpos });
                break;
            }

            case VX_MESSAGE_LAYER_SET_CAMERA_MODE:
            {
                uint8_t buf[1024];
                uint32_t bufpos = 0;

                encode_u32(buf, &bufpos, 0x124578ab); // magic
                encode_u8(buf, &bufpos, VX_MESSAGE_LAYER_SET_CAMERA_MODE);

                encode_string_u32(buf, &bufpos, msg.u.layer_set_camera_mode.layer_name);
                encode_string_u32(buf, &bufpos, msg.u.layer_set_camera_mode.mode);

                websocket_client_sendv(client, 1,
                                       (const void*[]) { buf },
                                       (uint64_t[]) { bufpos });
                break;
            }

            case VX_MESSAGE_BUFFER_DESTROY:
            {
                uint8_t buf[1024];
                uint32_t bufpos = 0;

                encode_u32(buf, &bufpos, 0x124578ab); // magic
                encode_u8(buf, &bufpos, VX_MESSAGE_BUFFER_DESTROY);

                encode_string_u32(buf, &bufpos, msg.u.buffer_destroy.layer_name);
                encode_string_u32(buf, &bufpos, msg.u.buffer_destroy.buffer_name);

                websocket_client_sendv(client, 1,
                                       (const void*[]) { buf },
                                       (uint64_t[]) { bufpos });
                break;
            }

            case VX_MESSAGE_BUFFER_REDRAW:
            {
                vx_lock();

                vx_serializer_t *snapshot = NULL;

                vx_layer_t *vl = vx_canvas_get_layer(vc, msg.u.buffer_redraw.layer_name);

                // NB: Don't create the buffer if it doesn't
                // exist. (We must be prepared for the buffer to have
                // been deleted out from beneath us.)
                vx_buffer_t *vb = vx_world_get_buffer_x(vl->world, msg.u.buffer_redraw.buffer_name, 0);

                if (vb)
                    snapshot = vx_serializer_copy(vb->serializer_front);

                vx_unlock();

                //////////////////////////////////////////////////////
                // now we can take our sweet time transmitting the snapshot.
                vx_serializer_t *last_snapshot = NULL;
                char *layerbuffer = malloc(strlen(msg.u.buffer_redraw.layer_name) + strlen(msg.u.buffer_redraw.buffer_name) + 2);
                sprintf(layerbuffer, "%s$%s", msg.u.buffer_redraw.layer_name, msg.u.buffer_redraw.buffer_name);

                zhash_get(last_snapshots, &layerbuffer, &last_snapshot);

                if (snapshot) {
                    // increment the reference count for everything in the current snapshot
                    zset_iterator_t zit;
                    zset_iterator_init(snapshot->resources, &zit);
                    vx_resource_t *resc;
                    while (zset_iterator_next(&zit, &resc)) {
                        int32_t refcnt = 0;
                        zhash_get(remote_resources_refcnt, &resc->id, &refcnt);
                        refcnt++;
                        zhash_put(remote_resources_refcnt, &resc->id, &refcnt, NULL, NULL);

                        if (refcnt == 1) {
                            // This was the first reference: we need to
                            // transmit this resource
                            switch(resc->type) {
                                case VX_RESOURCE_PROGRAM: {
                                    uint32_t buflen = strlen(resc->u.program.vertex_shader_src) +
                                        strlen(resc->u.program.fragment_shader_src) + 128;

                                    uint8_t buf[buflen];
                                    uint32_t bufpos = 0;

                                    encode_u32(buf, &bufpos, 0x124578ab); // magic
                                    encode_u8(buf, &bufpos, VX_MESSAGE_DEFINE_PROGRAM); // define program

                                    encode_u64(buf, &bufpos, resc->id);

                                    encode_string_u32(buf, &bufpos, resc->u.program.vertex_shader_src);
                                    encode_string_u32(buf, &bufpos, resc->u.program.fragment_shader_src);

                                    websocket_client_send_wrapper(layerbuffer, "program", client, buf, bufpos);

                                    break;
                                }

                                case VX_RESOURCE_ATTR_B:
                                    assert(0);

                                case VX_RESOURCE_ATTR_I32:
                                    assert(0);

                                case VX_RESOURCE_ATTR_F32:
                                {
                                    uint8_t *buf = malloc(resc->u.attr_f32.nelements * 4 + 1024);
                                    uint32_t bufpos = 0;

                                    encode_u32(buf, &bufpos, 0x124578ab); // magic
                                    encode_u8(buf, &bufpos,  VX_MESSAGE_DEFINE_VERTEX_ATTRIBUTE);
                                    encode_u64(buf, &bufpos, resc->id);
                                    encode_u8(buf, &bufpos,  resc->type);
                                    encode_u32(buf, &bufpos, resc->u.attr_f32.nelements);
                                    encode_u8(buf, &bufpos,  resc->u.attr_f32.dim);
                                    encode_u8(buf, &bufpos, 0); // padding

                                    for (int i = 0; i < resc->u.attr_f32.nelements; i++)
                                        encode_f32(buf, &bufpos, resc->u.attr_f32.data[i]);

                                    websocket_client_send_wrapper(layerbuffer, "attr_f32", client, buf, bufpos);
                                    free(buf);
                                    break;
                                }

                                case VX_RESOURCE_IDX_U8:
                                    assert(0);

                                case VX_RESOURCE_IDX_U16: {
                                    uint8_t *buf = malloc(resc->u.idx_u16.nelements * 2 + 1024);
                                    uint32_t bufpos = 0;

                                    encode_u32(buf, &bufpos, 0x124578ab); // magic
                                    encode_u8(buf, &bufpos, VX_MESSAGE_DEFINE_INDEX_ARRAY);

                                    encode_u64(buf, &bufpos, resc->id);
                                    encode_u8(buf, &bufpos, resc->type);

                                    encode_u32(buf, &bufpos, resc->u.idx_u16.nelements);

                                    for (int i = 0; i < resc->u.idx_u16.nelements; i++)
                                        encode_u16(buf, &bufpos, resc->u.idx_u16.data[i]);

                                    websocket_client_send_wrapper(layerbuffer, "idx_u16", client, buf, bufpos);
                                    free(buf);
                                    break;
                                }

                                case VX_RESOURCE_TEXTURE_U8X3: {
                                    image_u8x3_t *im = resc->u.texture_u8x3.im;

                                    uint8_t *buf = malloc(im->stride * im->height * 3 + 1024);
                                    uint32_t bufpos = 0;

                                    encode_u32(buf, &bufpos, 0x124578ab); // magic
                                    encode_u8(buf, &bufpos, VX_MESSAGE_DEFINE_TEXTURE);           // define texture

                                    encode_u64(buf, &bufpos, resc->id);

                                    encode_u32(buf, &bufpos, im->width);
                                    encode_u32(buf, &bufpos, im->height);
                                    encode_u32(buf, &bufpos, im->stride);
                                    encode_u32(buf, &bufpos, 1); // RGB encoding
                                    encode_u32(buf, &bufpos, resc->u.texture_u8x3.flags); // flags
                                    encode_u32(buf, &bufpos, 0); // compression format

                                    // XXX use _sendv
                                    encode_u32(buf, &bufpos, im->stride * im->height); // nbytes
                                    for (int y = 0; y < im->height; y++)
                                        encodeN(buf, &bufpos, &im->buf[y*im->stride], im->stride);

                                    websocket_client_send_wrapper(layerbuffer, "tex_u8x3", client, buf, bufpos);
                                    free(buf);
                                    break;
                                }

                                case VX_RESOURCE_TEXTURE_U8X4: {
                                    image_u8x4_t *im = resc->u.texture_u8x4.im;

                                    uint8_t *buf = malloc(im->stride * im->height * 4 + 1024);
                                    uint32_t bufpos = 0;

                                    encode_u32(buf, &bufpos, 0x124578ab); // magic
                                    encode_u8(buf, &bufpos, VX_MESSAGE_DEFINE_TEXTURE);           // define texture

                                    encode_u64(buf, &bufpos, resc->id);

                                    encode_u32(buf, &bufpos, im->width);
                                    encode_u32(buf, &bufpos, im->height);
                                    encode_u32(buf, &bufpos, im->stride);
                                    encode_u32(buf, &bufpos, 0); // RGBA encoding
                                    encode_u32(buf, &bufpos, resc->u.texture_u8x4.flags); // flags
                                    encode_u32(buf, &bufpos, 0); // compression format

                                    // XXX use _sendv
                                    encode_u32(buf, &bufpos, im->stride * im->height); // nbytes
                                    for (int y = 0; y < im->height; y++)
                                        encodeN(buf, &bufpos, &im->buf[y*im->stride], im->stride);

                                    websocket_client_send_wrapper(layerbuffer, "tex_u8x4", client, buf, bufpos);
                                    free(buf);
                                    break;
                                }

                                case VX_RESOURCE_TEXTURE_U8: {
                                    image_u8_t *im = resc->u.texture_u8.im;

                                    // XXX FIX ALLOC
                                    uint8_t *buf = malloc(im->stride * im->height + 1024);
                                    uint32_t bufpos = 0;

                                    encode_u32(buf, &bufpos, 0x124578ab); // magic
                                    encode_u8(buf, &bufpos, VX_MESSAGE_DEFINE_TEXTURE);           // define texture

                                    encode_u64(buf, &bufpos, resc->id);

                                    encode_u32(buf, &bufpos, im->width);
                                    encode_u32(buf, &bufpos, im->height);
                                    encode_u32(buf, &bufpos, im->stride);
                                    encode_u32(buf, &bufpos, 2); // U8 encoding
                                    encode_u32(buf, &bufpos, resc->u.texture_u8.flags); // flags

                                    if (0) {
                                        encode_u32(buf, &bufpos, 0); // compression format

                                        // XXX use _sendv
                                        encode_u32(buf, &bufpos, im->stride * im->height); // nbytes

                                        for (int y = 0; y < im->height; y++)
                                            encodeN(buf, &bufpos, &im->buf[y*im->stride], im->stride);
                                    } else {
                                        encode_u32(buf, &bufpos, 1); // compression format
                                        int inlength = im->height * im->stride;
                                        uint8_t *c = malloc(inlength * 2 + C5_PAD);
                                        int32_t clen = 0;
                                        c5(im->buf, inlength, c, &clen); // XXX encode directly into buf.

                                        encode_u32(buf, &bufpos, clen); // nbytes
                                        encodeN(buf, &bufpos, c, clen);

                                        free(c);
                                    }

                                    websocket_client_send_wrapper(layerbuffer, "tex_u8", client, buf, bufpos);
                                    free(buf);
                                    break;
                                }

                                default: {
                                    printf("unknown resource type: resc %p, type %d, id %d\n",
                                           resc, resc->type, (int) resc->id);
                                    assert(0);
                                }
                            }
                        }
                    }
                }

                //////////////////////////////////////////////////////
                // transmit the vx objects
                if (snapshot) {
                    uint8_t buf[1024];
                    uint32_t bufpos = 0;

                    encode_u32(buf, &bufpos, 0x124578ab); // magic
                    encode_u8(buf, &bufpos, VX_MESSAGE_BUFFER_REDRAW);
                    encode_string_u32(buf, &bufpos, msg.u.buffer_redraw.layer_name);
                    encode_string_u32(buf, &bufpos, msg.u.buffer_redraw.buffer_name);
                    encode_f32(buf, &bufpos, vb->draw_order);
                    encode_u32(buf, &bufpos, snapshot->data_len + 1);

                    uint8_t zero = 0;

                    websocket_client_sendv(client, 3,
                                           (const void*[]) { buf, snapshot->data, &zero },
                                           (uint64_t[]) { bufpos, snapshot->data_len, 1 });
                }

                //////////////////////////////////////////////////////
                if (last_snapshot != NULL) {
                    // decrement the reference count for everything in the last snapshot
                    zset_iterator_t zit;
                    zset_iterator_init(last_snapshot->resources, &zit);
                    vx_resource_t *resc;
                    while (zset_iterator_next(&zit, &resc)) {
                        int32_t refcnt = 0;
                        zhash_get(remote_resources_refcnt, &resc->id, &refcnt);
                        assert(refcnt >= 1);
                        refcnt--;
                        if (refcnt > 0) {
                            zhash_put(remote_resources_refcnt, &resc->id, &refcnt, NULL, NULL);
                        } else {
                            zhash_remove(remote_resources_refcnt, &resc->id, NULL, NULL);

                            uint8_t buf[64];
                            uint32_t bufpos = 0;

                            encode_u32(buf, &bufpos, 0x124578ab); // magic
                            switch (resc->type) {
                                case VX_RESOURCE_PROGRAM:
                                    encode_u8(buf, &bufpos, VX_MESSAGE_UNDEFINE_PROGRAM);
                                    break;
                                case VX_RESOURCE_ATTR_F32:
                                    encode_u8(buf, &bufpos, VX_MESSAGE_UNDEFINE_VERTEX_ATTRIBUTE);
                                    break;
                                case VX_RESOURCE_IDX_U16:
                                    encode_u8(buf, &bufpos, VX_MESSAGE_UNDEFINE_INDEX_ARRAY);
                                    break;
                                case VX_RESOURCE_TEXTURE_U8X4:
                                case VX_RESOURCE_TEXTURE_U8:
                                case VX_RESOURCE_TEXTURE_U8X3:
                                    encode_u8(buf, &bufpos, VX_MESSAGE_UNDEFINE_TEXTURE);
                                    break;

                                default:
                                    assert(0);
                                    break;
                            }
                            encode_u64(buf, &bufpos, resc->id);
                            websocket_client_send(client, buf, bufpos);
                        }
                    }
                }

                if (1) {
                    char *oldkey;
                    vx_serializer_t *oldvalue;
                    if (zhash_put(last_snapshots, &layerbuffer, &snapshot, &oldkey, &oldvalue)) {
                        free(oldkey);
                        vx_serializer_destroy(oldvalue);
                        assert(oldvalue == last_snapshot);
                    }
                }

                free(msg.u.buffer_redraw.layer_name);
                free(msg.u.buffer_redraw.buffer_name);
                break;
            }

            default:
            {
                printf("webvx: unhandled message type %d\n", msg.type);
                break;
            }
        }
    }

    canvasdef->on_destroy_canvas(vc, canvasdef->impl);

    websocket_client_destroy(client);

    zhash_destroy(remote_resources_refcnt);

    if (1) {
        zhash_iterator_t zit;
        zhash_iterator_init(last_snapshots, &zit);

        char *name;
        vx_serializer_t *serializer;
        while (zhash_iterator_next(&zit, &name, &serializer)) {
            free(name);
            vx_serializer_destroy(serializer);
        }

        zhash_destroy(last_snapshots);
    }

    return NULL;
}

static int on_connect(websocket_client_t *wsclient, struct httpd_basic_handler_results *bhres, void *user)
{
    struct webvx_canvas_definition *canvasdef = user;

    struct webvx_canvas_client_info *clientinfo = calloc(1, sizeof(struct webvx_canvas_client_info));
    clientinfo->canvasdef = canvasdef;
    clientinfo->wsclient = wsclient;
    clientinfo->webvx = canvasdef->webvx;

    vx_lock();
    zarray_add(canvasdef->client_infos, &clientinfo);
    vx_unlock();

    pthread_t worker_thread;
    pthread_create(&worker_thread, NULL, webvx_canvas_thread, clientinfo);

    fprintf(stderr,"ON_CONNECT, %s\n", wsclient->protocol);

/*    if (1) {
      int n = 1;
      if (setsockopt(wsclient->tx_fd, IPPROTO_TCP, TCP_NODELAY, &n, sizeof(n)) < 0)
      perror("could not set TCP_NODELAY");
      }
*/

    return 0;
}

// returns 1 if decode was successful.
static int basic_event_decode(vx_event_t *ev, const uint8_t *msg, uint32_t *inpos, int msg_len)
{
    memset(ev, 0, sizeof(vx_event_t));

    ev->client_utime = decode_u64(msg, inpos, msg_len);
    ev->utime = utime_now();
    ev->layer_name = decode_string_u32(msg, inpos, msg_len);
    for (int i = 0; i < 4; i++)
        ev->viewport[i] = decode_f32(msg, inpos, msg_len);
    for (int i = 0; i < 16; i++)
        ev->P[i] = decode_f64(msg, inpos, msg_len);
    for (int i = 0; i < 16; i++)
        ev->V[i] = decode_f64(msg, inpos, msg_len);

    ev->flags = decode_u32(msg, inpos, msg_len);

    return 1;
}

static int on_message(websocket_client_t *wsclient, const uint8_t *msg, int msg_len, void *user)
{
    struct webvx_canvas_definition *canvasdef = user;

    uint32_t inpos = 0;

    if (msg_len < 8 || decode_u32(msg, &inpos, msg_len) != 0x1186712a) {
        printf("--spurious message of length %d\n", msg_len);
        for (int i = 0; i < msg_len; i++) {
            if ((i % 16) == 0)
                printf("%04x: ", i);
            printf("%02x ", msg[i]);
            if ((i % 16) == 15)
                printf("\n");
        }
        printf("\n");
        return 0;
    }

    uint32_t code = decode_u32(msg, &inpos, msg_len);

    // find this clients canvas
    vx_canvas_t *vc = NULL;
    for (int i = 0; i < zarray_size(canvasdef->client_infos); i++) {
        struct webvx_canvas_client_info *cinfo;
        zarray_get(canvasdef->client_infos, i, &cinfo);
        if (cinfo->wsclient == wsclient) {
            // found it!
            vc = cinfo->vc;
        }
    }
    if (vc == NULL) { // uh oh
        printf("could not find canvas\n");
        return 1;
    }

    switch (code) {
        // CANVAS events
        case VX_EVENT_CANVAS_DRAW: {
            vx_event_t ev;
            ev.type = code;
            vx_canvas_dispatch_event(vc, &ev);
            break;
        }

        case VX_EVENT_CANVAS_ECHO: {
            vx_event_t ev;

            ev.type = code;
            ev.u.echo.nonce = decode_f64(msg, &inpos, msg_len);

            vx_canvas_dispatch_event(vc, &ev);

            break;
        }

        case VX_EVENT_READPIXELS: {
            uint64_t id = decode_u64(msg, &inpos, msg_len);
            uint32_t width = decode_u32(msg, &inpos, msg_len);
            uint32_t height = decode_u32(msg, &inpos, msg_len);
            uint32_t bytes_per_pixel = decode_u32(msg, &inpos, msg_len);

            vx_event_t ev;
            memset(&ev, 0, sizeof(ev));
            ev.type = VX_EVENT_READPIXELS;
            ev.u.readpixels.width = width;
            ev.u.readpixels.height = height;
            ev.u.readpixels.bytes_per_pixel = bytes_per_pixel;
            ev.u.readpixels.data = &msg[inpos];
            ev.u.readpixels.id = id;

            vx_canvas_dispatch_event(vc, &ev);
            printf("read pixels: %d, %d x %d\n", (int) id, width, height);
            break;
        }

        case VX_EVENT_CANVAS_CHANGE: {
            uint32_t width = decode_u32(msg, &inpos, msg_len);
            uint32_t height = decode_u32(msg, &inpos, msg_len);

            vx_event_t ev;
            memset(&ev, 0, sizeof(ev));
            ev.type = VX_EVENT_CANVAS_CHANGE;
            ev.u.canvas_change.width = width;
            ev.u.canvas_change.height = height;

            vx_canvas_dispatch_event(vc, &ev);

            break;
        }

        // LAYER events
        case VX_EVENT_MOUSE_UP:
        case VX_EVENT_MOUSE_DOWN:
        case VX_EVENT_MOUSE_CLICKED: {
            vx_event_t ev;
            basic_event_decode(&ev, msg, &inpos, msg_len);
            ev.type = code;

            ev.u.mouse_button.x = decode_f32(msg, &inpos, msg_len);
            ev.u.mouse_button.y = decode_f32(msg, &inpos, msg_len);
            ev.u.mouse_button.button = decode_u8(msg, &inpos, msg_len);

            vx_layer_t *layer = vx_canvas_get_layer(vc, ev.layer_name);
            vx_layer_dispatch_event(layer, &ev);
            free((char*) ev.layer_name);
            break;
        }

        case VX_EVENT_MOUSE_MOVED: {
            vx_event_t ev;
            basic_event_decode(&ev, msg, &inpos, msg_len);
            ev.type = code;

            ev.u.mouse_button.x = decode_f32(msg, &inpos, msg_len);
            ev.u.mouse_button.y = decode_f32(msg, &inpos, msg_len);

            vx_layer_t *layer = vx_canvas_get_layer(vc, ev.layer_name);
            vx_layer_dispatch_event(layer, &ev);
            free((char*) ev.layer_name);
            break;
        }

        case VX_EVENT_MOUSE_WHEEL: {
            vx_event_t ev;
            basic_event_decode(&ev, msg, &inpos, msg_len);
            ev.type = code;

            ev.u.mouse_wheel.amount = decode_f32(msg, &inpos, msg_len);

            vx_layer_t *layer = vx_canvas_get_layer(vc, ev.layer_name);
            vx_layer_dispatch_event(layer, &ev);
            free((char*) ev.layer_name);
            break;
        }

        case VX_EVENT_TOUCH_START:
        case VX_EVENT_TOUCH_MOVE:
        case VX_EVENT_TOUCH_TAP:
        case VX_EVENT_TOUCH_END: {
            vx_event_t ev;
            basic_event_decode(&ev, msg, &inpos, msg_len);
            ev.type = code;

            ev.u.touch.x = decode_f32(msg, &inpos, msg_len);
            ev.u.touch.y = decode_f32(msg, &inpos, msg_len);
            ev.u.touch.ntouch = decode_u32(msg, &inpos, msg_len);
            ev.u.touch.id = decode_u32(msg, &inpos, msg_len);

            vx_layer_t *layer = vx_canvas_get_layer(vc, ev.layer_name);
            vx_layer_dispatch_event(layer, &ev);
            free((char*) ev.layer_name);
            break;
        }

        case VX_EVENT_KEY_UP:
        case VX_EVENT_KEY_DOWN:
        case VX_EVENT_KEY_PRESSED: {
            vx_event_t ev;
            basic_event_decode(&ev, msg, &inpos, msg_len);
            ev.type = code;

            ev.u.key.key_code = decode_u32(msg, &inpos, msg_len);
            vx_layer_t *layer = vx_canvas_get_layer(vc, ev.layer_name);
            vx_layer_dispatch_event(layer, &ev);
            free((char*) ev.layer_name);
            break;
        }

        default: {
            printf("webvx.c: Unknown message type: %d\n", code);
            break;
        }

    }
    return 0;
}

static int on_disconnect(websocket_client_t *wsclient, void *user)
{
    struct webvx_canvas_definition *canvasdef = user;

    vx_lock();

    // find the webvx_canvas_client_info
    for (int i = 0; i < zarray_size(canvasdef->client_infos); i++) {
        struct webvx_canvas_client_info *cinfo;
        zarray_get(canvasdef->client_infos, i, &cinfo);
        if (cinfo->wsclient == wsclient) {
            // found it!

            vx_lock();
            cinfo->closed = 1;
            vx_unlock();

            // XXX BUG we won't wake up until the next time a buffer is swapped.
        }
    }

    vx_unlock();

    return 0;
}

webvx_t *webvx_create_server(int port, const char *rootdir, const char *default_file)
{
    webvx_t *webvx = calloc(1, sizeof(webvx_t));

    webvx->port = port;
    webvx->httpd = httpd_create();
    if (httpd_listen(webvx->httpd, port, 10, 0)) {
        fprintf(stderr, "Cannot create httpd on port %d, goodbye\n", port);
        exit(1);
    }

    char default_root[1024];
    default_root[0] = 0;

    if(rootdir == NULL) {
        char* envpath = april_util_root_path();

        snprintf(default_root, 1023, "%s/web/vx/", envpath);
        fprintf(stderr,
                "Using web folder %s\n",
                default_root);
    }

    if (1) {
        int sslport = port + 1000;

        ek_identity_t *identity = ek_identity_create("privkey.der", "cacert.der");
        fprintf(stderr, "Couldn't read privkey.der or cacert.der; not creating SSL server\n");

        if (identity) {
            ek_tls_server_t *server = ek_tls_server_create(identity);
            if (!server) {
                printf("Failed to create TLS server\n");
            } else {
                if (httpd_listen_tls(webvx->httpd, server, sslport, 10, 0))
                    printf("failed to create SSL listener on port %d\n", sslport);
                else
                    printf("SSL server created on port %d\n", sslport);
            }
        }
    }
    webvx->httpd->verbose = 0;
    webvx->canvas_definitions = zarray_create(sizeof(struct canvas_definition*));

    zhash_t *mime_types = zhash_create(sizeof(char*), sizeof(char*),
                                       zhash_str_hash, zhash_str_equals);
    char **mts = (char*[]) { "png", "image/png",
                             "jpg", "image/jpeg",
                             "jpeg", "image/jpeg",
                             "txt", "text/html",
                             "html", "text/html",
                             "mp3", "audio/mp3",
                             "zip", "application/x-compressed",
                             "js", "text/javascript",
                             "css", "text/css",
                             "pdf", "application/pdf",
                             NULL };

    for (int i = 0; mts[i] != NULL; i+=2) {
        char *k = strdup(mts[i]);
        char *v = strdup(mts[i+1]);
        zhash_put(mime_types, &k, &v, NULL, NULL);
    }

    zarray_t *default_files = zarray_create(sizeof(char*));
    char *s = strdup(default_file);
    zarray_add(default_files, &s);

    struct httpd_basic_handler_config *config = calloc(1, sizeof(struct httpd_basic_handler_config));
    config->host = NULL;
    config->port = 0;
    config->base_url = strdup("");
    config->default_files = default_files;
    config->mime_types = mime_types;

    if(default_root[0])
        config->base_path = strdup(default_root);
    else
        config->base_path = strdup(rootdir);

    httpd_add_handler(webvx->httpd, httpd_file_handler, config);

    return webvx;
}

void webvx_define_canvas(webvx_t *webvx, const char *name,
                         void (*on_create_canvas)(vx_canvas_t *vc, const char *name, void *impl),
                         void (*on_destroy_canvas)(vx_canvas_t *vc, void *impl),
                         void *impl)
{

    struct httpd_basic_handler_config *config = calloc(1, sizeof(struct httpd_basic_handler_config));
    config->host = NULL;
    config->port = 0;
    config->base_url = strdup("/webvx");
    config->base_path = NULL;
    config->default_files = NULL;
    config->mime_types = NULL;

    struct webvx_canvas_definition *canvasdef = calloc(1, sizeof(struct webvx_canvas_definition));
    canvasdef->webvx = webvx;
    canvasdef->on_create_canvas = on_create_canvas;
    canvasdef->on_destroy_canvas = on_destroy_canvas;
    canvasdef->wsserver = httpd_websocket_create(webvx->httpd, config,
                                                 on_connect, on_message, on_disconnect, canvasdef);

    canvasdef->impl = impl;
    canvasdef->client_infos = zarray_create(sizeof(struct webvx_client_info*));
    zarray_add(webvx->canvas_definitions, &canvasdef);
}
