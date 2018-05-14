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

#include "lcm/lcm.h"

#include "lcmtypes/raw_t.h"

#include "common/http_advertiser.h"
#include "common/floats.h"

#include "vx/vx.h"
#include "vx/webvx.h"
#include "vx/vxo_generic.h"

#include "april_velodyne.h"

#define DISPLAY_PACKETS 80
#define DISPLAY_SLICES 1000

typedef struct state state_t;
struct state
{
    lcm_t *lcm;
    vx_world_t *vw;
    webvx_t *webvx;

    zarray_t *vxos;
    int vxos_next;

    april_velodyne_t *velo;

    int slices;
    float xform[16];
    zarray_t *points;
    zarray_t *points_last;
    zarray_t *intensities;
};

void on_create_canvas(vx_canvas_t *vc, const char *name, void *impl)
{
    state_t *state = impl;

    vx_layer_t *vl = vx_canvas_get_layer(vc, "default");
    vx_layer_set_background_rgba(vl, vx_black, 0);

    vx_layer_set_world(vl, state->vw);
}

void on_destroy_canvas(vx_canvas_t *vc, void *impl)
{
    state_t *state = impl;

    printf("ON DESTROY CANVAS\n");
}

void on_slice(const zarray_t *pts, const zarray_t *ranges, const zarray_t *intensities, uint8_t mode, void *user)
{
    state_t *state = user;

    if(mode == VELO_FACTORY_LAST_RETURN) {
        zarray_add_all(state->points_last, pts);
        return;
    } else
        assert(mode == VELO_FACTORY_STRONGEST_RETURN);

    zarray_add_all(state->points, pts);

    for (int i = 0; i < zarray_size(ranges); i++) {
        float range;
        zarray_get(ranges, i, &range);

        uint8_t intensity;
        zarray_get(intensities, i, &intensity);

        if (range < 0.4)
            intensity = 255;

        float fintensity = intensity / 255.0f;
        zarray_add(state->intensities, &fintensity);
    }
}

void on_sweep(void *user)
{
    state_t *state = user;

    vx_object_t *vxo = vxo_points_pretty(
                            vx_resource_make_attr_f32_copy(
                                (float*) state->points->data,
                                3 * zarray_size(state->points),
                                3),
                            vx_resource_make_attr_f32_copy(
                                (float*) state->intensities->data,
                                zarray_size(state->intensities),
                                1)
                            );

    zarray_clear(state->points);
    zarray_clear(state->intensities);

    vx_buffer_t *vb = vx_world_get_buffer(state->vw, "points");
    vx_buffer_add_back(vb, vxo, NULL);
    vx_buffer_swap(vb);

    vxo = vxo_points(vx_resource_make_attr_f32_copy((float*) state->points_last->data,
                                                    3 * zarray_size(state->points_last),
                                                    3),
                     (float [4]){1,1,1,1},
                     1);
    zarray_clear(state->points_last);

    vb = vx_world_get_buffer(state->vw, "points_last");
    vx_buffer_add_back(vb, vxo, NULL);
    vx_buffer_swap(vb);
}

void on_velodyne_data(const lcm_recv_buf_t *rbuf, const char *channel, const raw_t *msg, void *user)
{
    state_t *state = user;

    if (msg->len != 1206) {
        printf("bad message length (%d)\n", msg->len);
        return;
    }

    april_velodyne_on_packet(state->velo,
                             msg->buf,
                             msg->len,
                             state->xform,
                             on_slice,
                             on_sweep,
                             state);
}

int main(int argc, char *argv[])
{
    state_t *state = calloc(1, sizeof(state_t));
    state->vw = vx_world_create();
    state->lcm = lcm_create(NULL);
    state->vxos = zarray_create(sizeof(vx_object_t*));

    state->velo = april_velodyne_create();
    state->points = zarray_create(sizeof(float[3]));
    state->points_last = zarray_create(sizeof(float[3]));
    state->intensities = zarray_create(sizeof(float));
    floats_mat44_identity(state->xform);

    float rot[16];
    floats_mat44_rotate_z(-M_PI/2, rot);
    float trans[16];
    float txyz[3] = {-1.5, 0, 1.73};    // Smartcarts sensor
    floats_mat44_translate(txyz, trans);
    floats_mat_AB(
                  trans, 4, 4,
                  rot, 4, 4,
                  state->xform, 4, 4);

    int port = 8200;
    if (1) {
        state->webvx = webvx_create_server(port,
                                           NULL, "index.html");
    }

    webvx_define_canvas(state->webvx, "mycanvas", on_create_canvas, on_destroy_canvas, state);
    http_advertiser_t * advertiser = http_advertiser_create(state->lcm, port, "VeloViewer", "View raw velodyne data");


    if (1) {
        vx_buffer_t *vb = vx_world_get_buffer(state->vw, "grid");
        vx_buffer_add_back(vb,
                           vxo_grid((float[]) { .5, .5, .5, .5 }, 1),
                           NULL);
        vx_buffer_swap(vb);
    }

    raw_t_subscribe(state->lcm, "VELODYNE_DATA", on_velodyne_data, state);

    while (1) {
        lcm_handle(state->lcm);
    }
}
