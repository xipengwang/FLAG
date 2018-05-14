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
#include <math.h>
#include <unistd.h>
#include <float.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include "vx/vx.h"
#include "vx/webvx.h"
#include "common/pam.h"

// avconv -f image2 -i /tmp/foo%08d.pam -r 30 foo.mp4 -y

typedef struct state state_t;
struct state
{
    vx_world_t *vw;
    webvx_t *webvx;

    vx_canvas_t *last_canvas;

    int outframes;
    // used to notify when we receive a frame
    pthread_mutex_t mutex;
    pthread_cond_t cond;
};

int on_canvas_event(vx_canvas_t *vc, const vx_event_t *ev, void *user)
{
    state_t *state = user;
    printf("canvas event\n");

    pam_t pam;
    memset(&pam, 0, sizeof(pam));
    pam.width = ev->u.readpixels.width;
    pam.height = ev->u.readpixels.height;
    pam.depth = 4;
    pam.maxval = 255;
    pam.type = PAM_RGB_ALPHA;
    pam.data = (uint8_t*) ev->u.readpixels.data;

    char path[1024];
    sprintf(path, "/tmp/foo%08d.pam", (int) ev->u.readpixels.id);
    int res = pam_write_file(&pam, path);
    if (res)
        printf("pam write failed\n");

    pthread_mutex_lock(&state->mutex);
    pthread_cond_broadcast(&state->cond);
    pthread_mutex_unlock(&state->mutex);

    state->outframes++;
    return 0;
}

void on_create_canvas(vx_canvas_t *vc, const char *name, void *impl)
{
    state_t *state = impl;
    state->last_canvas = vc;

    vx_layer_t *vl = vx_canvas_get_layer(vc, "default");

    vx_layer_set_world(vl, state->vw);
    vx_canvas_set_size(vc, 1920, 1080);
    vx_canvas_add_event_handler(vc, on_canvas_event, 0, state);
}

void on_destroy_canvas(vx_canvas_t *vc, void *impl)
{
    state_t *state = impl;

    if (vc == state->last_canvas)
        state->last_canvas = NULL;

    printf("ON DESTROY CANVAS\n");
}

int main(int argc, char *argv[])
{
    state_t *state = calloc(1, sizeof(state_t));
    pthread_mutex_init(&state->mutex, NULL);
    pthread_cond_init(&state->cond, NULL);

    state->vw = vx_world_create();

    if (1) {
        state->webvx = webvx_create_server(8200, NULL, "index.html");
    }

    webvx_define_canvas(state->webvx, "mycanvas", on_create_canvas, on_destroy_canvas, state);

    if (1) {
        vx_buffer_t *vb = vx_world_get_buffer(state->vw, "grid");
        vx_buffer_add_back(vb,
                           vxo_grid((float[]) { .5, .5, .5, .5 }, 1),
                           NULL);
        vx_buffer_swap(vb);
    }


    while (!state->last_canvas)
        sleep(1);

    int nframes = 100;

    for (int64_t id = 0; id < nframes; id++) {

        vx_buffer_t *vb = vx_world_get_buffer(state->vw, "text");
        vx_buffer_add_back(vb,
                           vxo_text(VXO_TEXT_ANCHOR_CENTER, "%d", (int) id),
                           NULL);
        vx_buffer_swap(vb);

        if (state->last_canvas) {
            vx_canvas_set_size(state->last_canvas, 1920, 1080);
            pthread_mutex_lock(&state->mutex);
            vx_canvas_readpixels(state->last_canvas, id);
            pthread_cond_wait(&state->cond, &state->mutex);
            pthread_mutex_unlock(&state->mutex);

        }
//        usleep(100E3);
    }

    while (state->outframes < nframes)
        sleep(1);

    return 0;
}
