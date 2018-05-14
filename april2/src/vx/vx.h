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

#ifndef _VX_H
#define _VX_H

#include <pthread.h>

#include "common/zarray.h"
#include "common/zhash.h"
#include "common/zset.h"
#include "common/matd.h"
#include "common/image_u8x3.h"
#include "common/image_u8x4.h"
#include "common/image_u8.h"
#include "common/c5.h"
#include "common/pam.h"
/*
  Resources:

    Resources, once created, are specified by uint64_t handles. All
    memory management is handled internal by vx.

    Functions named "_make_" create reference-counted objects and the
    user should not call destroy.

    Functions named "_create_" are the responsibility of the caller to destroy.

 Threading:

    VX calls must be non-blocking. We use a single global semaphore
(vx_lock() and vx_unlock()) to protect global state for these
non-blocking calls.  Slow operations (such as sending resources to a
display) are done on secondary threads.
*/

// NB: not all of these are implemented.
#define VX_RESOURCE_PROGRAM  1
#define VX_RESOURCE_ATTR_F32 2
#define VX_RESOURCE_ATTR_I32 3
#define VX_RESOURCE_ATTR_B   4
#define VX_RESOURCE_IDX_U8   5
#define VX_RESOURCE_IDX_U16  6
#define VX_RESOURCE_TEXTURE_U8X3 7
#define VX_RESOURCE_TEXTURE_U8   8
#define VX_RESOURCE_TEXTURE_U8X4 9

#define VX_GL_POINTS         0
#define VX_GL_LINES          1
#define VX_GL_LINE_LOOP      2
#define VX_GL_LINE_STRIP     3
#define VX_GL_TRIANGLES      4
#define VX_GL_TRIANGLE_STRIP 5
#define VX_GL_TRIANGLE_FAN   6

// by default, textures use NEAREST filtering and CLAMP.
#define VX_TEXTURE_DEFAULT    0
#define VX_TEXTURE_WRAP       1
#define VX_TEXTURE_MIN_LINEAR 2
#define VX_TEXTURE_MAX_LINEAR 4
#define VX_TEXTURE_MIPMAP     8

enum {
    // Note: events may be rate limited. This applies in particular to
    // _DRAW and _MOVE: you may not receive *every* message.

    // ======= These are LAYER events ========

    VX_EVENT_MOUSE_DOWN = 1001,
    VX_EVENT_MOUSE_MOVED = 1002,
    VX_EVENT_MOUSE_UP = 1003,
    VX_EVENT_MOUSE_CLICKED = 1004,
    VX_EVENT_MOUSE_WHEEL = 1005, // scroll wheel

    VX_EVENT_KEY_DOWN = 1010,
    VX_EVENT_KEY_PRESSED = 1011,
    VX_EVENT_KEY_UP = 1012,

    VX_EVENT_TOUCH_START = 1020,
    VX_EVENT_TOUCH_MOVE = 1021,
    VX_EVENT_TOUCH_END = 1022,
    VX_EVENT_TOUCH_TAP = 1023,

    // ======== These are CANVAS events ========
    //
    // Note that not all fields in vx_event are valid for canvas
    // events.

    VX_EVENT_CANVAS_DRAW = 1000,          // fires on every screen redraw
    VX_EVENT_CANVAS_ECHO = 1030,          // in response to an echo command
    VX_EVENT_CANVAS_CHANGE = 1040, // fires on orientation change and resize
    VX_EVENT_READPIXELS = 2000};

enum { VX_EVENT_FLAGS_SHIFT = 4, VX_EVENT_FLAGS_CTRL = 2, VX_EVENT_FLAGS_ALT = 1 };

enum { VX_KEY_ESC = 27, VX_KEY_BACKSPACE = 8, VX_KEY_UP = 38, VX_KEY_DOWN = 40, VX_KEY_LEFT = 37, VX_KEY_RIGHT = 39 };

enum { VX_CAMERA_CONTROLS_ZOOM = 1, VX_CAMERA_CONTROLS_PAN = 2, VX_CAMERA_CONTROLS_ROTATE = 4 };

typedef struct vx_event vx_event_t;
struct vx_event {
    int type; // one of: VX_EVENT_*

    uint64_t client_utime; // client utime
    uint64_t utime; // local time when event was received

    //////////////////////////////////
    // BELOW: only valid for LAYER events
    //////////////////////////////////

    const char *layer_name; // which layer did the event occur on?
    float viewport[4]; // viewport of the layer
    double P[16]; // projection matrix (i.e., perspective, orthographic is here.)
    double V[16]; // view matrix (i.e., camera matrix)
    int flags;

    //////////////////////////////////
    // ABOVE: only valid for LAYER events
    //////////////////////////////////

    union {
        struct {
            int width, height; // new size of canvas
        } canvas_change;

        struct {
            float x, y; // NB: in canvas coordinates.
            int button;
        } mouse_button; // mouse_down

        struct {
            float x, y; // NB: in canvas coordinates
        } mouse_moved;

        struct {
            int amount;
        } mouse_wheel;

        struct {
            int key_code;
        } key;

        struct {
            float x, y; //
            int ntouch;
            int id;
        } touch;

        struct {
            double nonce;
        } echo;

        struct {
            uint32_t width;
            uint32_t height;
            uint32_t bytes_per_pixel;
            const uint8_t *data;
            uint64_t id;
        } readpixels;
    } u;
};

typedef struct vx_resource vx_resource_t;
struct vx_resource {

    uint64_t id;

    void (*incref)(vx_resource_t *resc);
    void (*decref)(vx_resource_t *resc);

    int type; // e.g. VX_RESOURCE_FLOATS

    union {
        struct {
            char *vertex_shader_src, *fragment_shader_src;
        } program;

        // vertex attributes
        struct {
            float *data;
            int nelements, dim;
        } attr_f32;

        struct {
            int32_t *data;
            int nelements, dim;
        } attr_i32;

        // indices for ELEMENT_ARRAY
        struct {
            uint16_t *data;
            int nelements;
        } idx_u16;

        // textures
        struct {
            image_u8x3_t *im;
            int flags;
        } texture_u8x3;

        struct {
            image_u8x4_t *im;
            int flags;
        } texture_u8x4;

        struct {
            image_u8_t *im;
            int flags;
        } texture_u8;
    } u;

    uint32_t refcnt;
};

typedef struct vx_serializer vx_serializer_t;
struct vx_serializer {
    zset_t *resources; // vx_resource_t*

    uint8_t *data;
    int32_t data_alloc;
    int32_t data_len;
};

typedef struct vx_world vx_world_t;
struct vx_world {
    zhash_t *buffers; // char* => vx_buffer_t*

    zarray_t *layers; // which layers use us?
};

typedef struct vx_buffer vx_buffer_t;
struct vx_buffer {
    char *name;

    vx_world_t *world; // parent buffer
    int enabled;

    float draw_order;

    // protected by world->mutex.
    //
    // serializers contain both an opcode stream and a zset of the
    // resources that were referenced.
    vx_serializer_t *serializer_front;
    vx_serializer_t *serializer_back;
};

// Note that these contain the NAMES of the dirty layer and
// buffer. The actual objects may have since be deallocated (for
// example). vx_messages are the API between VX and the underlying
// transport (usually webvx).
enum { VX_MESSAGE_BUFFER_REDRAW = 1,
       VX_MESSAGE_BUFFER_DESTROY = 24,
       VX_MESSAGE_CANVAS_READ_PIXELS = 2,
       VX_MESSAGE_CANVAS_SET_SIZE = 3,
       VX_MESSAGE_CANVAS_SET_TITLE = 4, // set the title of the window that contains canvas
       VX_MESSAGE_CANVAS_ECHO = 23,
       VX_MESSAGE_LAYER_ENABLE_CAMERA_CONTROLS = 5,
       VX_MESSAGE_LAYER_SET_DRAW_ORDER = 6,
       VX_MESSAGE_LAYER_SET_BACKGROUND_COLOR = 7,
       VX_MESSAGE_LAYER_SET_POSITION = 8,
       VX_MESSAGE_LAYER_SET_ELU = 9,
       VX_MESSAGE_NOP = 10,
       VX_MESSAGE_DEBUG_MESSAGE = 11,
       VX_MESSAGE_DEFINE_PROGRAM = 12,
       VX_MESSAGE_UNDEFINE_PROGRAM = 13,
       VX_MESSAGE_DEFINE_VERTEX_ATTRIBUTE = 14,
       VX_MESSAGE_UNDEFINE_VERTEX_ATTRIBUTE = 15,
       VX_MESSAGE_DEFINE_INDEX_ARRAY = 16,
       VX_MESSAGE_UNDEFINE_INDEX_ARRAY = 17,
       VX_MESSAGE_DEFINE_TEXTURE = 18,
       VX_MESSAGE_UNDEFINE_TEXTURE = 19,
       VX_MESSAGE_DEFINE_NAMED_MATRIX = 20,
       VX_MESSAGE_UNDEFINE_NAMED_MATRIX = 21,
       VX_MESSAGE_LAYER_SET_CAMERA_MODE = 22,
};

struct vx_message
{
    int type;

    union {
        struct {
            double nonce;
        } canvas_echo;

        struct {
            uint64_t id;
        } canvas_readpixels;

        struct {
            uint32_t width, height;
        } canvas_setsize; // canvas_set_size

        struct {
            char *title;
        } canvas_set_title;

        struct {
            char *layer_name;
            uint32_t mask;
        } layer_enable_camera_controls;

        struct {
            char *layer_name;
            char *buffer_name;
        } buffer_redraw;

        struct {
            char *layer_name;
            char *buffer_name;
        } buffer_destroy;

        struct {
            char *layer_name;
            float draw_order;
        } layer_set_draw_order;

        struct {
            char *layer_name;
            float rgba[4];
            float mtime; // how many ms to animate?
        } layer_set_background_color;

        struct {
            char *layer_name;
            float position[4];
            float mtime; // how many ms to animate?
        } layer_set_position;

        struct {
            char *layer_name;
            double eye[3];
            double lookat[3];
            double up[3];
            float  mtime; // how many milliseconds to animate over?
        } layer_set_elu;

        struct {
            char *layer_name;
            char *mode;
        } layer_set_camera_mode;
    } u;
};

typedef struct vx_canvas vx_canvas_t;
struct vx_canvas {
    // protected by vx_lock().
    zhash_t *layers; // char* => vx_layer_t*

    zarray_t *event_handlers;

    // To support asynchronous data transfers, we assume most
    // vx_canvas implementations will have a worker thread that needs
    // to be woken up when a buffer swap is triggered.
    pthread_mutex_t mutex; // protects canvas_messages
    pthread_cond_t  cond;

    // protected by mutex/cond above.
    zarray_t *canvas_messages; // struct vx_canvas_message (by value)
};

typedef struct vx_layer vx_layer_t;
struct vx_layer {
    char *name;

    vx_world_t *world;

    vx_canvas_t *canvas;

    zarray_t *event_handlers; // struct vx_event_handler_info*
};

struct vx_canvas_event_handler_info
{
    // return 1 if the event has been consumed. Returning 0 will continue
    // event handling.
    int (*handler)(vx_canvas_t *vc, const vx_event_t *ev, void *user);

    int dispatch_order;
    void *user;
};

struct vx_layer_event_handler_info
{
    // return 1 if the event has been consumed. Returning 0 will continue
    // event handling.
    int (*handler)(vx_layer_t *vl, const vx_event_t *ev, void *user);

    int dispatch_order;
    void *user;
};


typedef struct vx_object vx_object_t;

struct vxo_generic_attribute
{
    char *name;
    vx_resource_t *resource;
};

struct vxo_generic_uniformf
{
    char *name;
    int nrows, ncols;
    float *data;
};

struct vxo_generic_texture
{
    char *name;
    vx_resource_t *resource;
};

struct vxo_generic_draw
{
    vx_resource_t *indices_resource; // can be NULL
    int command; // e.g. VX_GL_TRIANGLE_STRIP
    int first, count;
};

struct vx_object {

    void (*serialize)(vx_object_t *vxo, vx_serializer_t *outs);

    void (*incref)(vx_object_t *vxo);
    void (*decref)(vx_object_t *vxo);

    union {
        struct {
            zarray_t *objs;

            union {
                struct {
                    int coordspace; // 0 = standard model view.

                    // 1 = pixcoord model
                    // Where should the origin be as a multiple of the screen width and height?
                    float widthscale, heightscale;

                    // see vxo_chain.h enums.
                    uint8_t scale_mode;
                } pixcoords;

                struct {
                    uint8_t enable;
                } depth_test;
            } u;
        } chain;

        struct {
            double *data; // 16 doubles, row major order.
        } matrix;

        struct {
            vx_resource_t                *program_resource;

            int                           nattributes;
            struct vxo_generic_attribute *attributes;

            int                           nuniformfs;
            struct vxo_generic_uniformf   *uniformfs;

            int                           ntextures;
            struct vxo_generic_texture   *textures;

            int                           ndraws;
            struct vxo_generic_draw      *draws;

            void *impl;
        } generic;

        struct {
            vx_object_t *chain;
            double width, height;
        } text;
    } u;

    int refcnt; // use incref/decref methods!
};

void vx_lock();
void vx_unlock();

// to be called only by internal code
vx_canvas_t *vx_canvas_create();

vx_layer_t *vx_canvas_get_layer(vx_canvas_t *vc, const char *name);
void vx_canvas_set_world(vx_canvas_t *vc, vx_world_t *vw);

// triggers a readpixels on the canvas. The pixel data is sent back
// via a vx event.
void vx_canvas_readpixels(vx_canvas_t *vc, int64_t id);

void vx_canvas_set_size(vx_canvas_t *vc, uint32_t width, uint32_t height);

void vx_canvas_add_event_handler(vx_canvas_t *vc, int (*handler)(vx_canvas_t *vc, const vx_event_t *ev, void *user), int order, void *user);

void vx_canvas_dispatch_event(vx_canvas_t *vc, const vx_event_t *ev);

// used by implementations (like webvx) when an event occurs to dispatch an event
// with the layer listeners.
void vx_layer_dispatch_event(vx_layer_t *vl, const vx_event_t *ev);

void vx_layer_set_world(vx_layer_t *vl, vx_world_t *vw);

// this method is subsumed by _enable_camera_controls
//void vx_layer_enable_touch_camera_controls(vx_layer_t *vl, int enable_touch_camera_controls);
void vx_layer_set_background_rgba(vx_layer_t *vl, float rgba[4], float mtime);
void vx_layer_set_draw_order(vx_layer_t *vl, float draw_order);
void vx_layer_add_event_handler(vx_layer_t *vl, int (*handler)(vx_layer_t *vl, const vx_event_t *ev, void *user), int order, void *user);
void vx_layer_enable_camera_controls(vx_layer_t *vl, uint32_t mask);

// mode is one of 2D, 2F, 2.5D, 3D
void vx_layer_set_camera_mode(vx_layer_t *vl, const char *mode);

// used by implementations to send an echo request. The echo will
// return through the event pipe as a VX_EVENT_CANVAS_ECHO.
void vx_canvas_echo(vx_canvas_t *vc, double nonce);

//Sets the Page Title
void vx_canvas_set_title(vx_canvas_t *vc, const char *name);

// viewport[0]: left
// viewport[1]: top
// viewport[2]: width
// viewport[3]: height

// set the camera position for every layer using the specified world
void vx_world_set_elu(vx_world_t *vw, double eye[3], double lookat[3], double up[3], float mtime);
void vx_layer_set_elu(vx_layer_t *vl, double eye[3], double lookat[3], double up[3], float mtime);

void vx_layer_follow(vx_layer_t *vl, char *matrix_name, int mode);
void vx_layer_camera_mode(vx_layer_t *vl, char *camera_mode);

// NB: Not implemented
void vx_layer_camera_fit2D(vx_layer_t * vl, const float * xy0,
                           const float* xy1, uint8_t set_default);

vx_world_t *vx_world_create();

vx_buffer_t *vx_world_get_buffer_x(vx_world_t *vw, const char *name, int create);

// just calls _x with create=1
vx_buffer_t *vx_world_get_buffer(vx_world_t *vw, const char *name);

void vx_world_destroy_buffer(vx_world_t *vw, const char *name);

void vx_buffer_add_back(vx_buffer_t *vb, ...) __attribute__ ((__sentinel__));
void vx_buffer_add_back_destroy(vx_buffer_t *vb, ...) __attribute__ ((__sentinel__));
void vx_buffer_set_draw_order(vx_buffer_t *vb, float draw_order);
void vx_buffer_set_visible(vx_buffer_t *vb, uint32_t visible);

// viewport entries: if |val| <= 1, it is interpreted as a fraction of
//   the display width/height. left and top entries can be negative:
//   left=-0.3 means that it starts 0.3*width from the right edge of the screen.
//  if |val| > 1, the value is interpreted as a size in pixels. Left and top
//   can again be negative as above.
void vx_layer_set_position(vx_layer_t *vl, float position[4], float mtime);
void vx_buffer_swap(vx_buffer_t *vb);

// vx_resources are reference counted.
vx_resource_t *vx_resource_make_program(const char *vertex_shader_src, const char *fragment_shader_src);
vx_resource_t *vx_resource_make_attr_f32_copy(const float *data, int nelements, int dim);
vx_resource_t *vx_resource_make_idx_u16_copy(const uint16_t *data, int nelements);
vx_resource_t *vx_resource_make_texture_u8x3_copy(const image_u8x3_t *im, int flags);
vx_resource_t *vx_resource_make_texture_u8x4_copy(const image_u8x4_t *im, int flags);
vx_resource_t *vx_resource_make_texture_u8_copy(const image_u8_t *im, int flags);
vx_resource_t *vx_resource_make_texture_pam_copy(const pam_t *pam, int flags);

static inline int vx_resource_texture_get_height(vx_resource_t *resc)
{
    if (resc->type == VX_RESOURCE_TEXTURE_U8)
        return resc->u.texture_u8.im->height;
    else if (resc->type == VX_RESOURCE_TEXTURE_U8X3)
        return resc->u.texture_u8x3.im->height;
    else if (resc->type == VX_RESOURCE_TEXTURE_U8X4)
        return resc->u.texture_u8x4.im->height;
    assert(0);
    return 0;
}

static inline int vx_resource_texture_get_width(vx_resource_t *resc)
{
    if (resc->type == VX_RESOURCE_TEXTURE_U8)
        return resc->u.texture_u8.im->width;
    else if (resc->type == VX_RESOURCE_TEXTURE_U8X3)
        return resc->u.texture_u8x3.im->width;
    else if (resc->type == VX_RESOURCE_TEXTURE_U8X4)
        return resc->u.texture_u8x4.im->width;
    assert(0);
    return 0;
}

static inline int vx_resource_texture_get_stride(vx_resource_t *resc)
{
    if (resc->type == VX_RESOURCE_TEXTURE_U8)
        return resc->u.texture_u8.im->stride;
    else if (resc->type == VX_RESOURCE_TEXTURE_U8X3)
        return resc->u.texture_u8x3.im->stride;
    else if (resc->type == VX_RESOURCE_TEXTURE_U8X4)
        return resc->u.texture_u8x4.im->stride;
    assert(0);
    return 0;
}

static inline int vx_resource_texture_get_bpp(vx_resource_t *resc)
{
    if (resc->type == VX_RESOURCE_TEXTURE_U8)
        return 1;
    else if (resc->type == VX_RESOURCE_TEXTURE_U8X3)
        return 3;
    else if (resc->type == VX_RESOURCE_TEXTURE_U8X4)
        return 4;
    assert(0);
    return 0;
}

static inline void vx_object_incref(vx_object_t *vxo)
{
    vxo->incref(vxo);
}

static inline void vx_resource_incref(vx_resource_t *resc)
{
    resc->incref(resc);
}

// if refcnt is zero, the resource will be immediately destroyed.
void vx_resource_decref(vx_resource_t *resc);

//////////////////////////////////////////////////////////
// These are commands available to any vx object, independent of the
// transport in use. Particular display implementations are separately
// responsible for transmitting resources.

enum {
    VX_SERIALIZER_MODEL_PUSH = 1,
    VX_SERIALIZER_MODEL_POP = 2,
    VX_SERIALIZER_MODEL_MULTIPLY = 3,
    VX_SERIALIZER_PIXCOORD_PUSH = 4,
    VX_SERIALIZER_PIXCOORD_POP = 5,
    VX_SERIALIZER_DEPTH_TEST_PUSH = 6,
    VX_SERIALIZER_DEPTH_TEST_POP = 7,
    VX_SERIALIZER_EXECUTE_PROGRAM = 100
};


// copies the encoded byte stream and increment the reference count
vx_serializer_t *vx_serializer_copy(vx_serializer_t *in);
void vx_serializer_destroy(vx_serializer_t *outs);

void vx_serializer_u8(vx_serializer_t *outs, uint8_t v);
void vx_serializer_u32(vx_serializer_t *outs, uint32_t v);
void vx_serializer_u64(vx_serializer_t *outs, uint64_t v);
void vx_serializer_float(vx_serializer_t *outs, float v);
void vx_serializer_double(vx_serializer_t *outs, double v);
void vx_serializer_opcode(vx_serializer_t *outs, uint8_t opcode);
void vx_serializer_resource(vx_serializer_t *outs, vx_resource_t *resc);
void vx_serializer_string(vx_serializer_t *outs, const char *s);

void vx_util_unproject(const double winxyz[3], matd_t *P, matd_t *V, const float viewport[4], double objxyz[3]);
void vx_util_mouse_event_compute_ray(const vx_event_t *ev, double r0[3], double r1[3]);
void vx_util_touch_event_compute_ray(const vx_event_t *ev, double r0[3], double r1[3]);
void vx_util_ray_intersect_plane(const double r0[3], const double r1[3], const double ABCD[4], double xyz[3]);

#include "vx_colors.h"
#include "vxo_chain.h"
#include "vxo_matrix.h"
#include "vxo_robot.h"
#include "vxo_points.h"
#include "vxo_image.h"
#include "vxo_sphere.h"
#include "vxo_text.h"
#include "vxo_lines.h"
#include "vxo_triangles.h"
#include "vxo_circle.h"
#include "vx_console.h"
#include "vxo_grid.h"
#include "vxo_square.h"
#include "vxo_mesh_model.h"
#include "vxo_box.h"

#endif
