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

#include "common/doubles.h"

#include "../vx.h"
#include "../webvx.h"

#include "common/mesh_model.h"

typedef struct state state_t;
struct state
{
    vx_world_t *vw;
    webvx_t *webvx;
    vx_console_t *vx_console;

    mesh_model_t *model;

    int select_mode;
    double select_center[2];
    double select_r;
};

void redraw(state_t *state);

int on_event(vx_layer_t *vl, const vx_event_t *ev, void *user)
{
	state_t *state = user;

    double r0[3], r1[3];
    vx_util_mouse_event_compute_ray(ev, r0, r1);

    double xyz[3];
    vx_util_ray_intersect_plane(r0, r1, (double[]) { 0, 0, 1, 0 }, xyz);

    if (state->select_mode) {
        if (ev->flags == VX_EVENT_FLAGS_CTRL && ev->type == VX_EVENT_MOUSE_DOWN) {

            state->select_center[0] = xyz[0];
            state->select_center[1] = xyz[1];
            state->select_r = 1;
        }

        if (ev->flags == VX_EVENT_FLAGS_CTRL && ev->type == VX_EVENT_MOUSE_MOVED) {
            state->select_r = doubles_distance(xyz, state->select_center, 2);
        }

        if (1) {
            vx_buffer_t *vb = vx_world_get_buffer(state->vw, "selection");
            vx_buffer_add_back(vb,
                               vxo_matrix_translate(state->select_center[0],
                                                    state->select_center[1],
                                                    0),
                               vxo_matrix_scale(state->select_r),
                               vxo_circle_solid((float[]) { 1, 1, 0, 1 }),
                               NULL);
            vx_buffer_swap(vb);
        }
    }

    return 0;
}

void on_console_command(vx_console_t *vc, vx_layer_t *vl, const char *cmd, void *user)
{
    state_t *state = user;

    zarray_t *toks = str_split(cmd, " ");

    if (zarray_size(toks) >= 1) {
        char *t0;
        zarray_get(toks, 0, &t0);

        if (!strcmp(t0, "select-on")) {
            state->select_mode = 1;
        }

        if (!strcmp(t0, "select-off")) {
            state->select_mode = 0;
        }

        if (!strcmp(t0, "select-delete-all-except")) {
            mesh_model_t *model = state->model;

            int tris_kept = 0;
            int tris_deleted = 0;

            for (int chunkidx = 0; chunkidx < zarray_size(model->chunks); chunkidx++) {
                struct mesh_model_chunk *chunk;
                zarray_get(model->chunks, chunkidx, &chunk);

                if (chunk->indices) {
                    printf("index mode....\n");

                    for (int triidx = 0; triidx < zarray_size(chunk->indices); triidx++) {
                        uint16_t *tri;
                        zarray_get_volatile(chunk->indices, triidx, &tri);

                        int inside = 0;
                        for (int i = 0; i < 3; i++) {
                            float *v;
                            zarray_get_volatile(chunk->vertices, tri[i], &v);
                            double dist = sqrt(pow(v[0] - state->select_center[0], 2) +
                                               pow(v[1] - state->select_center[1], 2));

                            if (dist < state->select_r)
                                inside = 1;
                        }

                        if (inside)
                            tris_kept++;
                        else
                            tris_deleted++;
                    }

                    assert(0); // not fully implemented.

                } else {

                    for (int vidx = 0; vidx < zarray_size(chunk->vertices); vidx+=3) {
                        int inside = 0;

                        for (int i = 0; i < 3; i++) {
                            float *v;
                            zarray_get_volatile(chunk->vertices, vidx + i, &v);

                            double dist = sqrt(pow(v[0] - state->select_center[0], 2) +
                                               pow(v[1] - state->select_center[1], 2));

                            if (dist < state->select_r)
                                inside = 1;
                        }

                        if (!inside) {
                            for (int i = 0; i < 3; i++)
                                zarray_remove_index(chunk->vertices, vidx, 0);

                            if (chunk->normals)
                                for (int i = 0; i < 3; i++)
                                    zarray_remove_index(chunk->normals, vidx, 0);

                            if (chunk->texcoords)
                                for (int i = 0; i < 2; i++)
                                    zarray_remove_index(chunk->texcoords, vidx, 0);

                            tris_deleted++;
                            vidx -= 3;
                        } else {
                            tris_kept++;
                        }
                    }

                    if (zarray_size(chunk->vertices) == 0) {
                        // we emptied the chunk. Delete it.
                        zarray_remove_index(model->chunks, chunkidx, 0);
                        chunkidx--;
                    }
                }
            }

            printf("kept %d, deleted %d\n", tris_kept, tris_deleted);
            redraw(state);
        }

        if (!strcmp(t0, "clear")) {
            vx_console_clear(vc);
        }

        if (!strcmp(t0, "save") && zarray_size(toks) >= 2) {
            char *t1;
            zarray_get(toks, 1, &t1);

            if (mesh_model_save_obj(state->model, t1))
                vx_console_printf(vc, "error saving\n");
        }

        if (!strcmp(t0, "rotate-z") && zarray_size(toks) >= 2) {
            char *t1;
            zarray_get(toks, 1, &t1);
            double v = atof(t1) * M_PI / 180;

            double M[16];
            doubles_mat44_rotate_z(v, M);
            doubles_print_mat(M, 4, 4, "%15f");

            mesh_model_transform(state->model, M);
            redraw(state);
        }

        if (!strcmp(t0, "rotate-x") && zarray_size(toks) >= 2) {
            char *t1;
            zarray_get(toks, 1, &t1);
            double v = atof(t1) * M_PI / 180;

            double M[16];
            doubles_mat44_rotate_x(v, M);
            mesh_model_transform(state->model, M);
            redraw(state);
        }

        if (!strcmp(t0, "rotate-y") && zarray_size(toks) >= 2) {
            char *t1;
            zarray_get(toks, 1, &t1);
            double v = atof(t1) * M_PI / 180;

            double M[16];
            doubles_mat44_rotate_y(v, M);
            mesh_model_transform(state->model, M);
            redraw(state);
        }

        if (!strcmp(t0, "scale") && zarray_size(toks) >= 2) {
            char *t1;
            zarray_get(toks, 1, &t1);
            double v = atof(t1);

            double M[16];
            doubles_mat44_identity(M);
            M[0*4 + 0] = v;
            M[1*4 + 1] = v;
            M[2*4 + 2] = v;
            mesh_model_transform(state->model, M);
            redraw(state);
        }

        if (!strcmp(t0, "translate") && zarray_size(toks) >= 4) {
            double txyz[3];
            for (int i = 0; i < 3; i++) {
                char *t;
                zarray_get(toks, i+1, &t);
                txyz[i] = atof(t);
            }

            double M[16];
            doubles_mat44_translate(txyz, M);
            mesh_model_transform(state->model, M);
            redraw(state);
        }

        if (!strcmp(t0, "chunk-delete") && zarray_size(toks) >= 2) {
            char *t1;
            zarray_get(toks, 1, &t1);

            for (int cidx = 0; cidx < zarray_size(state->model->chunks); cidx++) {
                struct mesh_model_chunk *chunk;
                zarray_get(state->model->chunks, cidx, &chunk);

                if (!strcmp(chunk->name, t1)) {
                    zarray_remove_index(state->model->chunks, cidx, 0);
                    vx_console_printf(vc, "deleted");

                    vx_buffer_t *vb = vx_world_get_buffer(state->vw, chunk->name);
                    vx_buffer_swap(vb);
                }
            }
            redraw(state);
        }

        if (!strcmp(t0, "chunk-rgba") && zarray_size(toks) >= 6) {
            char *t1;
            zarray_get(toks, 1, &t1);

            float rgba[4];
            for (int i = 0; i < 4; i++) {
                char *t;
                zarray_get(toks, i+2, &t);
                rgba[i] = atof(t);
            }


            for (int cidx = 0; cidx < zarray_size(state->model->chunks); cidx++) {
                struct mesh_model_chunk *chunk;
                zarray_get(state->model->chunks, cidx, &chunk);

                if (!strcmp(chunk->name, t1)) {
                    memcpy(chunk->rgba, rgba, 4 * sizeof(float));
                }
            }
            redraw(state);
        }
    }
}

zarray_t* on_console_tab(vx_console_t *vc, vx_layer_t *vl, const char *cmd, void *user)
{
    state_t *state = user;
    zarray_t *completions = zarray_create(sizeof(char*));

    const char *commands[] = { "clear",
                               "rotate-z ",
                               "rotate-x ",
                               "rotate-y ",
                               "scale ",
                               "translate ",
                               "chunk-delete ",
                               "chunk-rgba ",
                               "select-on",
                               "select-off",
                               "select-delete-all-except",
                               "save ",
                               NULL };
    for (int idx = 0; commands[idx]; idx++) {
        if (str_starts_with(commands[idx], cmd)) {
            char *s = strdup(commands[idx]);
            zarray_add(completions, &s);
        }
    }

    return completions;
}

void on_create_canvas(vx_canvas_t *vc, const char *name, void *impl)
{
    state_t *state = impl;

    vx_layer_t *vl = vx_canvas_get_layer(vc, "default");

    vx_layer_set_world(vl, state->vw);

    vx_console_setup_layer(state->vx_console, vl);

    vx_layer_add_event_handler(vl, on_event, 0, state);
//    vx_layer_camera_mode(vl, "3");
}

void on_destroy_canvas(vx_canvas_t *vc, void *impl)
{
    state_t *state = impl;

    printf("ON DESTROY CANVAS\n");
}

void redraw(state_t *state)
{
    mesh_model_t *model = state->model;

    static zarray_t *last_buffer_names = NULL;

    if (!last_buffer_names)
        last_buffer_names = zarray_create(sizeof(char*));

    for (int cidx = 0; cidx < zarray_size(last_buffer_names); cidx++) {
        char *buffer_name;
        zarray_get(last_buffer_names, cidx, &buffer_name);

        vx_world_destroy_buffer(state->vw, buffer_name);
        free(buffer_name);
    }

    zarray_clear(last_buffer_names);

    // put each chunk into a separate buffer
    for (int cidx = 0; cidx < zarray_size(model->chunks); cidx++) {
        struct mesh_model_chunk *chunk;
        zarray_get(model->chunks, cidx, &chunk);
        mesh_model_t *m = mesh_model_create();
        zarray_add(m->chunks, &chunk);

        char bufname[1024];
        sprintf(bufname, "chunk-%d", cidx);
        vx_buffer_t *vb = vx_world_get_buffer(state->vw, bufname);
        vx_buffer_add_back(vb,
                           vxo_mesh_model_create(m),
                           NULL);
        vx_buffer_swap(vb);

        char *s = strdup(bufname);
        zarray_add(last_buffer_names, &s);
    }
}

int main(int argc, char *argv[])
{
    state_t *state = calloc(1, sizeof(state_t));
    state->vw = vx_world_create();
    state->vx_console = vx_console_create(vx_world_get_buffer(state->vw, "console"),
                                              on_console_command,
                                              on_console_tab,
                                              state);

    state->select_mode = 1;

    if (1) {
        int port = 8200;
        state->webvx = webvx_create_server(port, NULL, "index.html");
        printf("listening on port %d\n", port);
    }

    webvx_define_canvas(state->webvx, "mycanvas", on_create_canvas, on_destroy_canvas, state);

    if (1) {
        vx_buffer_t *vb = vx_world_get_buffer(state->vw, "grid");
        vx_buffer_add_back(vb,
                           vxo_grid((float[]) { .5, .5, .5, .5 }, 1),
                           NULL);
        vx_buffer_swap(vb);
    }

    struct mesh_model_create_params params;
    mesh_model_create_params_init(&params);
//    params.combine_materials = 0;

    // state->model = mesh_model_create_from_obj_params(argv[1], &params);
    printf("%s\n", argv[1]);
    // char *mtlpath = strdup(argv[1]);
    // int len = strlen(mtlpath);
    // mtlpath[len-5] = '2';
    // mtlpath[len-3] = 'm';
    // mtlpath[len-2] = 't';
    // mtlpath[len-1] = 'l';
    // printf("%s\n", mtlpath);

    state->model = mesh_model_create_from_obj(argv[1]);
    // state->model = mesh_model_create_with_mtl(argv[1], mtlpath);
    mesh_model_normals_from_faces(state->model);

    // float rgba[] = {0,1,0,0.5};
    // // mesh_model_paint(state->model, rgba);
    // struct mesh_model_chunk *chunk = NULL;//calloc(1, sizeof(struct mesh_model_chunk));
    // for(int i=0; i<zarray_size(state->model->chunks); i++){
    //     zarray_get_volatile(state->model->chunks, i, &chunk);
    //     // printf("%s before %f %f %f \n", __func__, chunk->rgba[0],chunk->rgba[1],chunk->rgba[2]);
    //     // memcpy(chunk->rgba, rgba, 4 * sizeof(float));
    //     // zarray_add(chunks_array, chunk);
    //     // chunk->shininess = 10;
    //     printf("%s after %f %f %f \n", __func__, chunk->rgba[0],chunk->rgba[1],chunk->rgba[2]);

    // }
    
    if (!state->model) {
        printf("load of model failed.\n");
        exit(1);
    }

    vx_buffer_t *vb = vx_world_get_buffer(state->vw, "markers");
    vx_buffer_add_back(vb,
                       vxo_chain(
                                 vxo_matrix_scale(1),
                                 // vxo_matrix_rotatex(M_PI/2),
                                 vxo_mesh_model_create(state->model),                       
                                 NULL),
                       NULL);
        vx_buffer_swap(vb);

    // for (int cidx = 0; cidx < zarray_size(state->model->chunks); cidx++) {
    //     struct mesh_model_chunk *chunk;
    //     zarray_get(state->model->chunks, cidx, &chunk);
    //     if (!chunk->name) {
    //         char buf[1024];
    //         sprintf(buf, "chunk-%d", cidx);
    //         chunk->name = strdup(buf);
    //     }
    // }

//    mesh_model_normals_from_faces(state->model);

//    on_console_command(NULL, NULL, "scale .01", state);
//    on_console_command(NULL, NULL, "rotate-x 90", state);
//    mesh_model_save_obj(state->model, "/tmp/rescaled");
//    printf("done\n");

    redraw(state);

    while (1) {
        sleep(1);
    }
}
