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
#include <stdlib.h>
#include <pthread.h>
#include "sim.h"
#include "common/floats.h"
#include "common/doubles.h"
#include "common/time_util.h"
#include "common/io_util.h"
#include "lcmtypes/pose_t.h"
#include "lcmtypes/laser_t.h"
#include "lcmtypes/diff_drive_t.h"
#include "vx/vx.h"
#include "vx/webvx.h"
#include "lcm/lcm.h"
#include "common/stype.h"
#include "common/getopt.h"

#include "sim.h"

enum { MODE_NONE, MODE_DELETE, MODE_BOXES, MODE_WALLS, MODE_MOUND,
       MODE_MOVE_PICK, MODE_MOVE_MOVING };

typedef struct state state_t;
struct state {
    lcm_t *lcm;

    vx_world_t *vw;
	webvx_t *webvx;
    vx_console_t *vx_console;

    int mode;

    double mound_height;
    double mound_width;

    sim_world_t *simworld;

    getopt_t *gopt;
    int surface_idx; // currently selected surface

    float  box_M[16];
    float  box_rgba[4];
    float  box_shininess;

    double snapto;

    sim_object_t *pick;
};

void rebuild(state_t *state);

int get_num_surfaces(state_t *state)
{
    int cnt = 0;
    for (int i = 0; i < zarray_size(state->simworld->objects); i++) {
        sim_object_t *so;
        zarray_get(state->simworld->objects, i, &so);
        if (so->type == SIM_OBJECT_SURFACE)
            cnt++;
    }
    return cnt;
}

sim_object_t *get_surface(state_t *state, int idx)
{
    for (int i = 0; i < zarray_size(state->simworld->objects); i++) {
        sim_object_t *so;
        zarray_get(state->simworld->objects, i, &so);
        if (so->type == SIM_OBJECT_SURFACE)
            idx--;

        if (idx < 0)
            return so;
    }
    return NULL;
}

void on_console_command(vx_console_t *vc, vx_layer_t *vl, const char *cmd, void *user)
{
    state_t *state = user;

    printf("console: %s\n", cmd);
    zarray_t *toks = str_split(cmd, " ");

    if (zarray_size(toks) == 0)
        goto cleanup;

    char *tok;
    zarray_get(toks, 0, &tok);

    if (!strcmp(tok, "delete")) {
        state->mode = MODE_DELETE;
        state->pick = NULL;
        goto cleanup;
    }

    if (!strcmp(tok, "save") && zarray_size(toks) == 2) {
        char *path;
        zarray_get(toks, 1, &path);

        uint32_t len = 0;
        stype_encode_object(NULL, &len, state->simworld->stype, state->simworld);

        uint8_t *buf = calloc(1, len);
        len = 0;
        stype_encode_object(buf, &len, state->simworld->stype, state->simworld);

        if (ioutils_write_file(path, (char*) buf, len))
            vx_console_printf(state->vx_console, "save failed\n");

        free(buf);
        goto cleanup;
    }

    if (!strcmp(tok, "load") && zarray_size(toks) == 2) {
        char *path;
        zarray_get(toks, 1, &path);

        state->simworld = sim_world_create_from_file(path);

        if (!state->simworld) {
            vx_console_printf(state->vx_console, "file load failed\n");
            return;
        }

        rebuild(state);

        goto cleanup;
    }

    if (!strcmp(tok, "model") && zarray_size(toks) == 2) {
        char *path;
        zarray_get(toks, 1, &path);

        mesh_model_t *mm = mesh_model_create_from_obj(path);
        // UNFINISHED
    }

    if (!strcmp(tok, "clear")) {
        vx_console_clear(vc);
        goto cleanup;
    }

    if (!strcmp(tok, "move")) {
        state->mode = MODE_MOVE_PICK;
        state->pick = NULL;
        goto cleanup;
    }

    if (!strcmp(tok, "wall")) {
        state->mode = MODE_WALLS;
        state->pick = NULL;
        goto cleanup;
    }

    if (!strcmp(tok, "mound") && zarray_size(toks) == 1) {
        state->mode = MODE_MOUND;
        state->pick = NULL;
        goto cleanup;
    }

    if (!strcmp(tok, "mound-height") && zarray_size(toks) == 2) {
        char *val;
        zarray_get(toks, 1, &val);
        state->mound_height = atof(val);
        goto cleanup;
    }

    if (!strcmp(tok, "mound-width") && zarray_size(toks) == 2) {
        char *val;
        zarray_get(toks, 1, &val);
        state->mound_width = atof(val);
        goto cleanup;
    }

    if (!strcmp(tok, "surface-create")) {
        sim_object_t *so = sim_object_surface_create((double[]) { -5, -5},
                                                     (double[]) { 5, 5 },
                                                     .25, // pitch
                                                     3*get_num_surfaces(state),   // initial height,
                                                     (float[]) { .2, .2, .2, 1 },
                                                     0.3);
        zarray_add(state->simworld->objects, &so);
        state->surface_idx = get_num_surfaces(state) - 1;

        rebuild(state);
        goto cleanup;
    }

    if (!strcmp(tok, "surface-index") && zarray_size(toks) == 2) {
        char *val;
        zarray_get(toks, 1, &val);
        state->surface_idx = atoi(val);
        goto cleanup;
    }

    if (!strcmp(tok, "box")) {
        if (zarray_size(toks) == 4) {
            for (int i = 0; i < 3; i++) {
                char *t;
                zarray_get(toks, i+1, &t);
                double v = atof(t);
                state->box_M[4*i+i] = v;
            }
        }
        state->mode = MODE_BOXES;
        state->pick = NULL;
        goto cleanup;
    }

    if (!strcmp(tok, "box-color") && zarray_size(toks) == 4) {
        for (int i = 0; i < 3; i++) {
            char *t;
            zarray_get(toks, i+1, &t);
            double v = atof(t);
            state->box_rgba[i] = v;
        }
        state->mode = MODE_BOXES;
        state->pick = NULL;
        goto cleanup;
    }

    if (!strcmp(tok, "box-shininess") && zarray_size(toks) == 2) {
        char *t;
        zarray_get(toks, 1, &t);
        double v = atof(t);
        state->box_shininess = v;
        state->mode = MODE_BOXES;
        state->pick = NULL;
        goto cleanup;
    }

    if (!strcmp(tok, "snapto") && zarray_size(toks) == 2) {
        char *t;
        zarray_get(toks, 1, &t);
        double v = atof(t);
        state->snapto = v;
        goto cleanup;
    }

    vx_console_printf(vc, "unknown command '%s'", cmd);

   goto cleanup;

  cleanup:
    zarray_vmap(toks, free);
    zarray_destroy(toks);

}

zarray_t* on_console_tab(vx_console_t *vc, vx_layer_t *vl, const char *cmd, void *user)
{
    state_t *state = user;
    zarray_t *completions = zarray_create(sizeof(char*));

    const char *commands[] = { "clear",
                               "delete",
                               "move",
                               "box ", "box-color ", "box-shininess ",
                               "wall",
                               "mound", "mound-height ", "mound-width ",
                               "surface-create ",
                               "surface-index ",
                               "save ",
                               "load ",
                               "snapto ",
                               NULL };
    for (int idx = 0; commands[idx]; idx++) {
        if (str_starts_with(commands[idx], cmd)) {
            char *s = strdup(commands[idx]);
            zarray_add(completions, &s);
        }
    }

    return completions;
}

void rebuild(state_t *state)
{
    printf("rebuild %d objects\n", zarray_size(state->simworld->objects));

    vx_buffer_t *vb = vx_world_get_buffer(state->vw, "model");
    for (int i = 0; i < zarray_size(state->simworld->objects); i++) {
        sim_object_t *so;
        zarray_get(state->simworld->objects, i, &so);

        if (!so->world.valid || so->world.mesh_last_version != so->mesh_version) {
            vx_object_t *vxo = so->world.mesh_last_vxo;

            if (so->world.valid)
                vxo->decref(vxo);

            so->world.mesh_last_version = so->mesh_version;
            vxo = vxo_mesh_model_create(so->mesh);
            so->world.mesh_last_vxo = vxo;
            vxo->incref(vxo);
            so->world.valid = 1;
            printf("created mesh %p\n", vxo);
        }

        vx_buffer_add_back(vb,
                           vxo_matrix(so->T),
                           so->world.mesh_last_vxo,
                           NULL);
    }

    vx_buffer_swap(vb);
}

int on_event(vx_layer_t *vl, const vx_event_t *ev, void *user)
{
	state_t *state = user;

    double r0[3], r1[3];
    vx_util_mouse_event_compute_ray(ev, r0, r1);

    double plane_xyz[3];
    vx_util_ray_intersect_plane(r0, r1, (double[]) { 0, 0, 1, 0 }, plane_xyz);

    double dir[3] = { r1[0] - r0[0], r1[1] - r0[1], r1[2] - r0[2] };
    doubles_normalize(dir, 3, dir);

    sim_object_t *new_pick = NULL;
    double pick_range = sim_world_ray_cast(state->simworld, r0, dir, DBL_MAX,
                                           (sim_object_t*[]) { state->pick }, 1, &new_pick);


    double new_pick_xyz[3];
    if (pick_range < DBL_MAX) {
        for (int i = 0; i < 3; i++)
            new_pick_xyz[i] = r0[i] + pick_range * dir[i];
    } else {
        for (int i = 0; i < 3; i++)
            new_pick_xyz[i] = plane_xyz[i];
    }

//    new_pick_xyz[0] += state->snapto / 2;
//    new_pick_xyz[1] += state->snapto / 2;

    if (state->mode == MODE_BOXES) {

        double snapto = state->snapto;
        if (snapto < 1e-8)
            snapto = 1e-8;

        for (int i = 0; i < 2; i++) {
            if (new_pick_xyz[i] > 0)
                new_pick_xyz[i] += snapto / 2;
            else
                new_pick_xyz[i] -= snapto / 2;
            new_pick_xyz[i] -= fmod(new_pick_xyz[i], snapto);
        }
    }

    if (1) {
		vx_buffer_t *vb = vx_world_get_buffer(state->vw, "pick");
		vx_buffer_add_back(vb,
                           vxo_matrix_translate(new_pick_xyz[0], new_pick_xyz[1], new_pick_xyz[2]),
                           vxo_matrix_scale(0.15),
                           vxo_sphere_solid((float[]) { 0.25, .25, 1, 1 }),
                           NULL);
		vx_buffer_swap(vb);
	}


    if (ev->type == VX_EVENT_MOUSE_MOVED) {
        if (state->mode == MODE_MOVE_MOVING) {
            assert(state->pick);

            state->pick->T[0*4 + 3] = new_pick_xyz[0];
            state->pick->T[1*4 + 3] = new_pick_xyz[1];
            state->pick->T[2*4 + 3] = new_pick_xyz[2];

            rebuild(state);
        }
    }

    if (ev->type == VX_EVENT_MOUSE_CLICKED) {

        switch (state->mode) {
            case MODE_DELETE: {

                if (new_pick) {
                    zarray_remove_value(state->simworld->objects, &new_pick, 1);
                    rebuild(state);
                    printf("deleted\n");
                    if (new_pick->stype->destroy)
                        new_pick->stype->destroy(new_pick->stype, new_pick);
                } else {
                    printf("no object detected\n");
                }
                break;
            }

            case MODE_MOVE_MOVING: {
                state->mode = MODE_MOVE_PICK;
                state->pick = NULL;
                break;
            }

            case MODE_MOVE_PICK: {
                state->pick = new_pick;
                if (state->pick)
                    state->mode = MODE_MOVE_MOVING;
                break;
            }

            case MODE_BOXES: {
                double T[16];
                doubles_mat44_translate(new_pick_xyz, T);

                float M[16];
                floats_mat44_translate((float[]) { 0, 0, .5}, M);

                float MM[16];
                floats_mat_AB(state->box_M, 4, 4, M, 4, 4, MM, 4, 4);

                sim_object_t *so = sim_object_box_create(T, MM, state->box_rgba, state->box_shininess);
                zarray_add(state->simworld->objects, &so);
                rebuild(state);
                break;
            }

            case MODE_MOUND: {
                sim_object_t *so = get_surface(state, state->surface_idx);;
                assert(so);

                double pxyz[3];
                doubles_mat44_inv_transform_xyz(so->T, plane_xyz, pxyz);

                sim_object_surface_mound(so, pxyz[0], pxyz[1], state->mound_height, state->mound_width);
                rebuild(state);

                break;
            }


        }
    }

    if (ev->type == VX_EVENT_MOUSE_MOVED) {
        double r0[3], r1[3];
        vx_util_mouse_event_compute_ray(ev, r0, r1);

        double xyz[3];
        vx_util_ray_intersect_plane(r0, r1, (double[]) { 0, 0, 1, 0 }, xyz);
    }

    return 0;
}

void on_create_canvas(vx_canvas_t *vc, const char *name, void *impl)
{
    state_t *state = impl;

    vx_layer_t *vl = vx_canvas_get_layer(vc, "default");

    vx_layer_set_world(vl, state->vw);

    vx_layer_add_event_handler(vl, on_event, 0, state);

    vx_console_setup_layer(state->vx_console, vl);
}

void on_destroy_canvas(vx_canvas_t *vc, void *impl)
{
	state_t *state = impl;

	printf("ON DESTROY CANVAS\n");
}

int main(int argc, char *argv[])
{
    setlinebuf(stderr);
    setlinebuf(stdout);

    sim_stype_init();

    state_t *state = calloc(1, sizeof(state_t));
    state->gopt = getopt_create();
    getopt_add_bool(state->gopt, 'h', "help", 0, "Show this help screen");
    getopt_add_int(state->gopt, 'p', "port", "1999", "Select http port");

    if (!getopt_parse(state->gopt, argc, argv, 1) || getopt_get_bool(state->gopt, "help")) {
        getopt_do_usage(state->gopt);
        exit(-1);
    }

    floats_mat44_identity(state->box_M);
    state->box_rgba[0] = .5;
    state->box_rgba[1] = .5;
    state->box_rgba[2] = .5;
    state->box_rgba[3] = 1;
    state->box_shininess = 0.5;

    state->lcm = lcm_create(NULL);
	state->vw = vx_world_create();
    state->vx_console = vx_console_create(vx_world_get_buffer(state->vw, "console"),
                                              on_console_command,
                                              on_console_tab,
                                              state);

    int port = getopt_get_int(state->gopt, "port");
    state->webvx = webvx_create_server(port, NULL, "index.html");
    printf("listening on port %d\n", port);

    state->mode = MODE_NONE;

    state->mound_height = 0.2;
    state->mound_width = 3;
    state->snapto = 0.0001;

    const zarray_t *extraargs = getopt_get_extra_args(state->gopt);

    if (zarray_size(extraargs) == 0) {
        // create a default world
        state->simworld = sim_world_create();

        if (1) {
            sim_object_t *so = sim_object_surface_create((double[]) { -10, -10},
                                                         (double[]) { 10, 10 },
                                                         1, // pitch
                                                         0,   // initial height,
                                                         (float[]) { 0, .3, 0, 1 },
                                                         0.3);

            zarray_add(state->simworld->objects, &so);
        }

        rebuild(state);

    } else {
        char *path;
        zarray_get(extraargs, 0, &path);

        state->simworld = sim_world_create_from_file(path);

        if (!state->simworld) {
            printf("load failed\n");
            exit(1);
        }

        rebuild(state);
    }

    webvx_define_canvas(state->webvx, "mycanvas", on_create_canvas, on_destroy_canvas, state);

    if (1) {
		vx_buffer_t *vb = vx_world_get_buffer(state->vw, "grid");
		vx_buffer_add_back(vb,
                           vxo_grid((float[]) { .5, .5, .5, .5 }, 1),
                           NULL);
		vx_buffer_swap(vb);
	}


    while (1) {
        sleep(1);
    }
}
