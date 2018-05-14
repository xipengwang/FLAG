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

#include "lcm/lcm.h"
#include "lcmtypes/laser_t.h"

#include "common/getopt.h"
#include "common/http_advertiser.h"

#include "vx/vx.h"
#include "vx/webvx.h"
#include "vx/vxo_generic.h"

typedef struct state state_t;
struct state
{
    lcm_t *lcm;
    vx_world_t *vw;
    webvx_t *webvx;
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

void on_laser_t(const lcm_recv_buf_t *rbuf, const char *channel, const laser_t *msg, void *user)
{
    state_t *state = user;

    zarray_t *points = zarray_create(sizeof(float[2]));
    zarray_t *intensities = zarray_create(sizeof(float));

    for(int i = 0; i < msg->nranges; i++)
    {
        double theta = msg->rad0 + i * msg->radstep;

        double ct = cos(theta);
        double st = sin(theta);

        float xy[2] ={ msg->ranges[i] * ct, msg->ranges[i] * st};
        zarray_add(points, xy);
    }

    vx_resource_t *points_resc = vx_resource_make_attr_f32_copy((float*) points->data, zarray_size(points)*2, 2);

    vx_buffer_t * vb = vx_world_get_buffer(state->vw, "points");
    vx_buffer_add_back(vb,
                       vxo_points(points_resc, (float[]){0,0,1,1}, 2),
                       NULL);
    vx_buffer_swap(vb);

    vb = vx_world_get_buffer(state->vw, "lines");
    vx_buffer_add_back(vb,
                       vxo_line_strip(points_resc, (float[]){0,.2,.8,1}, 1),
                       NULL);
    vx_buffer_swap(vb);

    zarray_destroy(points);
    zarray_destroy(intensities);
}

int main(int argc, char *argv[])
{
    setlinebuf (stdout);

    state_t *state = calloc(1, sizeof(state_t));
    state->vw = vx_world_create();
    state->lcm = lcm_create(NULL);
    getopt_t * gopt = getopt_create();

    getopt_add_bool(gopt, 'h',"help", 0,"Show this");
    getopt_add_string(gopt, 'c', "channel", "HOKUYO_LIDAR", "LC channel name (used if channel-map is empty)");

    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt,"help")) {
        printf("Usage: %s [options]\n\n", argv[0]);
        getopt_do_usage(gopt);
        return 0;
    }

    int port = 8222;
    state->webvx = webvx_create_server(port, NULL, "index.html");
    webvx_define_canvas(state->webvx, "mycanvas", on_create_canvas, on_destroy_canvas, state);
    http_advertiser_t * advertiser = http_advertiser_create(state->lcm, port, "VeloViewer", "View raw velodyne data");

    if (0) {
        vx_buffer_t *vb = vx_world_get_buffer(state->vw, "grid");
        vx_buffer_add_back(vb,
                           vxo_grid((float[]) { .5, .5, .5, .5 }, 1),
                           NULL);
        vx_buffer_swap(vb);
    }
    if (1) {
        vx_buffer_t *vb = vx_world_get_buffer(state->vw, "rings");
        for (int i = 1; i < 200; i++)
        {
            vx_buffer_add_back(vb,
                               vxo_matrix_scale(i * 0.10),
                               vxo_circle_line((float [4]) {.5, .5, .5, .5}, 1),
                               NULL);
        }
        for (int i = 1; i < 20; i++)
        {
            vx_buffer_add_back(vb,
                               vxo_matrix_scale(i),
                               vxo_circle_line((float [4]) {1, 1, 1, 1}, 1),
                               NULL);
        }
        vx_buffer_swap(vb);
    }

    laser_t_subscribe(state->lcm, getopt_get_string(gopt, "channel"), on_laser_t, state);

    while (1) {
        lcm_handle(state->lcm);
    }
}
