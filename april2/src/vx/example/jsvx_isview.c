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

#include <unistd.h>
#include "../vx.h"
#include "../webvx.h"

#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

#include "common/time_util.h"
#include "common/image_u8x3.h"

typedef struct state state_t;
struct state
{
    image_source_t *isrc;
    vx_world_t *vw;
    webvx_t *webvx;
};

void on_create_canvas(vx_canvas_t *vc, const char *name, void *impl)
{
    state_t *state = impl;

    printf("ON CREATE CANVAS\n");
    vx_layer_set_world(vx_canvas_get_layer(vc, "default"), state->vw);

}

void on_destroy_canvas(vx_canvas_t *vc, void *impl)
{
    state_t *state = impl;

    printf("ON DESTROY CANVAS\n");
}

int main(int argc, char *argv[])
{
    if (argc != 2) {
        printf("Must provide a image source URL.\n");
        exit(-1);
    }

    int port = 8400;

    state_t *state = calloc(1, sizeof(state_t));
    state->vw = vx_world_create();
    state->webvx = webvx_create_server(8400, NULL, "index.html");
    state->isrc = image_source_open(argv[1]);
    assert(state->isrc);

    webvx_define_canvas(state->webvx, "mycanvas", on_create_canvas, on_destroy_canvas, state);
    printf("Listening on port %d\n", port);

    state->isrc->start(state->isrc);

    for (int iter = 0; 1; iter++) {
        image_source_data_t isdata;
        memset(&isdata, 0, sizeof(image_source_data_t));

        if (state->isrc->get_frame(state->isrc, &isdata)) {
            printf("fail\n");
            break;
        }

        image_u8x3_t *im = image_convert_u8x3(&isdata);

        printf("frame %d\n", iter);

        vx_buffer_t *vb = vx_world_get_buffer(state->vw, "robot");
//        vx_buffer_add_back(vb, vxo_matrix_rotatez(utime_now() / 1.0E6), vxo_robot(), NULL);
        vx_buffer_add_back(vb, vxo_chain(vxo_matrix_scale3(.01, -.01, 1), vxo_image_u8x3(im, 0), NULL), NULL);

        vx_buffer_swap(vb);

        image_u8x3_destroy(im);

//        usleep(1000*1000);
        state->isrc->release_frame(state->isrc, &isdata);
    }
}
