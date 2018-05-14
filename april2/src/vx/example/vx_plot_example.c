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
#include <lcm/lcm.h>
#include <math.h>
#include <poll.h>
#include <stdlib.h>
#include <inttypes.h>

#include "vx/vx.h"
#include "vx/webvx.h"
#include "common/time_util.h"
#include "common/getopt.h"

#include "vx/vx_plot.h"

double r2()
{
    return (double)rand() / (double)RAND_MAX ;
}


typedef struct state state_t;
struct state
{
    lcm_t *lcm;
    vx_world_t *vw;
    webvx_t *webvx;
    getopt_t *gopt;

    // For global plot
    zarray_t *plots; //vx_plot_t *

};


void generate_data(state_t *state)
{
    static int64_t counter0, counter1;
    vx_plot_t *p = NULL;
    zarray_get(state->plots, 1, &p);
    float data[] = {counter1, r2()};
    vx_plot_add_data(p, "live_data_limited_memory", data, 2);
    float data3[] = {counter1, r2()};
    vx_plot_add_data(p, "live_data_limited_memory2", data3, 2);
    float data4[] = {counter1++, r2()};
    vx_plot_add_data(p, "live_data_limited_memory3", data4, 2);

    
    zarray_get(state->plots, 0, &p);
    float data2[] = {counter0++, r2()};
    vx_plot_add_data(p, "live_data", data2, 2);
}

void render_plots(state_t *state)
{
    vx_buffer_t *vb = vx_world_get_buffer(state->vw, "plots");

    for (int i = 0; i < zarray_size(state->plots); i++) {
        vx_plot_t *p = NULL;
        zarray_get(state->plots, i, &p);
        vx_plot_render(p, vb);
    }

    vx_buffer_swap(vb);
}

static int on_event(vx_layer_t *vl, const vx_event_t *ev, void *impl)
{

    state_t *state = impl;

    //int ret = 0;
    for (int i = 0; i < zarray_size(state->plots); i++) {
        vx_plot_t *p = NULL;
        zarray_get(state->plots, i, &p);
        if (vx_plot_on_event(p, vl, ev, state)) {
            break;
        }
    }

    render_plots(state);
    return 0;
}

void on_create_canvas(vx_canvas_t *vc, const char *name, void *impl)
{
    state_t *state = impl;

    vx_canvas_set_title(vc, "Plot Test");

    vx_layer_t *vl = vx_canvas_get_layer(vc, "default");
    vx_layer_set_world(vl, state->vw);
    vx_layer_add_event_handler(vl, on_event, 0, state);
}

void on_destroy_canvas(vx_canvas_t *vc, void *impl)
{

}


int main(int argc, char *argv[])
{
    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Show this help");

    int parse_success = getopt_parse(gopt, argc, argv, 1);
    if (!parse_success || getopt_get_bool(gopt, "help")) {
        getopt_do_usage(gopt);
    }

    int port = 4096;
    printf("running on port %d \n", port);
    printf("%s Click on plot to interact. WSAD to move. QE for zoom.\n", __func__);
    state_t *state = calloc(1, sizeof(state_t));
    state->vw = vx_world_create();
    state->webvx = webvx_create_server(port, NULL, "index.html");
    state->gopt = gopt;
    state->plots = zarray_create(sizeof(vx_plot_t *));

    //Plot 1 
    vx_plot_t *p1 = vx_plot_create(VXO_PIXCOORDS_BOTTOM_LEFT, (double[]){10, -460, 450, 450});
    vx_plot_add_xlabel(p1, "Time");
    vx_plot_add_ylabel(p1, "Value");
    char title[32];
    sprintf(title, "Plot 1 - Data Stream - No memory limit");
    vx_plot_add_title(p1, title);
    float refline[] = {0,0.5};
    vx_plot_add_ranges(p1, 0, 1, 0, 1);
    vx_plot_init_data(p1,  "error_thresh", refline, 2, vx_red, PLOT_REFLINE, 2, 0);
    vx_plot_init_data(p1,  "live_data", NULL, 0, vx_blue, PLOT_LINE, 2, 0);
    zarray_add(state->plots, &p1);


    //Plot 2 
    p1 = vx_plot_create_with_color(VXO_PIXCOORDS_TOP_RIGHT, (double[]){-600, 10, 600, 600},
                                   vx_white, vx_black, vx_black);
    vx_plot_add_xlabel(p1, "Time");
    vx_plot_add_ylabel(p1, "Value");
    char title2[32];
    sprintf(title2, "Plot 2 - Data Stream - Limited Memory");
    vx_plot_add_title(p1, title2);
    float refline2[] = {0,0.5};
    vx_plot_add_ranges(p1, 0, 1, 0, 1);
    vx_plot_init_data(p1,  "error_thresh", refline2, 2, vx_red, PLOT_REFLINE, 2, 0);
    vx_plot_init_data(p1,  "live_data_limited_memory", NULL, 0, vx_blue, PLOT_LINE, 2, 40);
    vx_plot_init_data(p1,  "live_data_limited_memory2", NULL, 0, vx_yellow, PLOT_LINE, 2, 40);
    vx_plot_init_data(p1,  "live_data_limited_memory3", NULL, 0, vx_green, PLOT_LINE, 2, 40);
    
    zarray_add(state->plots, &p1);


    webvx_define_canvas(state->webvx, "mycanvas", on_create_canvas, on_destroy_canvas, state);
    int64_t last_render_utime;
    int64_t counter = 0;
    while (1) {
        int64_t now = utime_now();

        if ((now - last_render_utime) > 250000) {
            last_render_utime = utime_now();
            
            generate_data(state);
            render_plots(state);
        }
    }

    return 0;
}
