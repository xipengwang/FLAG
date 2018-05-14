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
#include <inttypes.h>
#include "lcm/lcm.h"
#include "vx/vx.h"
#include "vx/webvx.h"
#include "common/http_advertiser.h"
#include "common/time_util.h"
#include "common/getopt.h"
#include "lcmtypes/image_t.h"
#include "common/image_convert.h"

typedef struct state state_t;
struct state
{
    lcm_t *lcm;
    vx_world_t *vw;
    webvx_t *webvx;
    int frame_count;

    pthread_mutex_t frame_mutex;
    image_u8x4_t * last_frame;
};

int on_event(vx_layer_t *vl, const vx_event_t *ev, void *user)
{
    state_t *state = user;

    if (ev->type == VX_EVENT_KEY_UP) {

        if (ev->u.key.key_code == ' ') {
            bool taken = false;
            pthread_mutex_lock(&state->frame_mutex);
            if(state->last_frame){
                taken = true;
                char buf[256];
                sprintf(buf, "%" PRId64 ".pnm", utime_now());
                printf("Saved image to %s\n", buf);
                image_u8x4_write_pnm(state->last_frame, buf);
            }
            pthread_mutex_unlock(&state->frame_mutex);

            if(taken) {
                vx_buffer_t *vb = vx_world_get_buffer(state->vw, "flash");
                vx_buffer_add_back(vb, vxo_pixcoords(VXO_PIXCOORDS_SCALE_MODE_MAX, VXO_PIXCOORDS_CENTER,vxo_square_solid((float[]){1,1,1,1}),NULL),NULL);
                vx_buffer_swap(vb);
                usleep(1e5);
                vx_buffer_swap(vb);
            }
            return 1;
        }
    }

    return 0;
}

void on_create_canvas(vx_canvas_t *vc, const char *name, void *impl)
{
    state_t *state = impl;

    vx_canvas_set_title(vc, "Image Viewer");

    vx_layer_t *vl = vx_canvas_get_layer(vc, "default");
    vx_layer_set_background_rgba(vl, vx_black, 0);

    vx_layer_set_world(vl, state->vw);
    vx_layer_add_event_handler(vl, on_event, 0, state);
}

void on_destroy_canvas(vx_canvas_t *vc, void *impl)
{
    state_t *state = impl;

    printf("ON DESTROY CANVAS\n");
}

void on_image_t(const lcm_recv_buf_t *rbuf, const char *channel, const image_t *msg, void *user)
{
    state_t *state = user;

    image_u8x4_t *im = image_t_to_image_u8x4(msg);


    pthread_mutex_lock(&state->frame_mutex);
    {
        if(state->last_frame)
            image_u8x4_destroy(state->last_frame);
        state->last_frame = im;
    }
    pthread_mutex_unlock(&state->frame_mutex);

    vx_buffer_t *vb = vx_world_get_buffer(state->vw, "image");

    vx_object_t *chain = vxo_chain(
        vxo_matrix_scale(10.0 / im->width),
        vxo_matrix_translate(-im->width/2, -im->height/2, 0),
        vxo_image_u8x4(im, 0),
        NULL);
    vx_buffer_add_back(vb, chain, NULL);
    vx_buffer_swap(vb);
}

int main(int argc, char *argv[])
{
    printf("Image Viewer\n");

    int exit_code = 0;

    // Parse input
    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Show this help screen");
    getopt_add_string(gopt, 'c', "channel", "IMAGE", "image_t channel");
    getopt_add_int(gopt, 'p', "port", "8201", "web server port");

    if (!getopt_parse(gopt, argc, argv, 1)) {
        fprintf(stderr, "Usage: %s [-c <image channel>]\n", argv[0]);
        getopt_do_usage(gopt);
        exit_code = 1;
        goto cleanup;
    }

    // Set up state
    state_t *state = calloc(1, sizeof(state_t));
    state->vw = vx_world_create();
    state->lcm = lcm_create(NULL);

    // Set up webvx
    int port = getopt_get_int(gopt, "port");
    state->webvx = webvx_create_server(port, NULL, "index.html");
    webvx_define_canvas(state->webvx, "mycanvas", on_create_canvas, on_destroy_canvas, state);
    http_advertiser_t * ha = http_advertiser_create(state->lcm, port, "ImageViewer", "View images");

    image_t_subscribe(state->lcm, getopt_get_string(gopt, "channel"), on_image_t, state);

    while (1)
    {
        lcm_handle(state->lcm);
    }


  cleanup:
    getopt_destroy(gopt);
    return exit_code;
}
