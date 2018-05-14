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

#include "common/time_util.h"

typedef struct state state_t;
struct state
{
    vx_world_t *vw;
    webvx_t *webvx;
};

void on_create_canvas(vx_canvas_t *vc, const char *name, void *impl)
{
    state_t *state = impl;

    printf("ON CREATE CANVAS\n");
    vx_layer_t *vl = vx_canvas_get_layer(vc, "default");

    vx_layer_set_world(vl, state->vw);
}

void on_destroy_canvas(vx_canvas_t *vc, void *impl)
{
    state_t *state = impl;

    printf("ON DESTROY CANVAS\n");
}

int main(int argc, char *argv[])
{
    state_t *state = calloc(1, sizeof(state_t));
    state->vw = vx_world_create();
    state->webvx = webvx_create_server(1234, NULL, "index.html");
    webvx_define_canvas(state->webvx, "mycanvas", on_create_canvas, on_destroy_canvas, state);

    //image_u8x3_t *im = image_u8x3_create_from_pnm("earth.pnm");
    //vx_resource_t *texture_resource = vx_resource_make_texture_u8x3_copy(im, VX_TEXTURE_WRAP | VX_TEXTURE_MIPMAP);
    //image_u8x3_destroy(im);
    printf("port:1234\n");

    for (int iter = 0; 1; iter++) {
        if (1) {
            vx_buffer_t *vb = vx_world_get_buffer(state->vw, "robot");
            vx_buffer_add_back(vb, vxo_matrix_rotatez(utime_now() / 1.0E6), vxo_robot_solid((float []){1.0, 1.0, 1.0, 1.0}), NULL);
//            vx_buffer_add_back(vb, vxo_matrix_rotatez(0 * utime_now() / 1.0E6), vxo_sphere_textured(texture_resource), NULL);
            vx_buffer_add_back(vb, vxo_matrix_rotatez(0 * utime_now() / 1.0E6), vxo_sphere_solid((float[]) { 1.0, 0.5, 0, 1.0}), NULL);
            vx_buffer_swap(vb);
        }

        if (1) {
            char buf[1024];
            sprintf(buf, "<<#ffff00,monospaced-bold-36>>iteration %d\n<<#ffffff>>line 2", iter);

            vx_buffer_t *vb = vx_world_get_buffer(state->vw, "text");
            vx_buffer_add_back(vb, vxo_pixcoords(VXO_PIXCOORDS_BOTTOM_LEFT,
                                                 VXO_PIXCOORDS_SCALE_MODE_ONE,
                                                 vxo_text(VXO_TEXT_ANCHOR_BOTTOM_LEFT, buf),
                                                 NULL),
                               NULL);
            vx_buffer_add_back(vb, vxo_text(VXO_TEXT_ANCHOR_CENTER, "ABC"), NULL);
            vx_buffer_swap(vb);

        }

        usleep(100*1000);
    }
}
